// Copyright (c) 2026, OpenEmu Team
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the OpenEmu Team nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY OpenEmu Team ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL OpenEmu Team BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#import "OEGIPUSBDeviceHandler.h"
#import "OEDeviceDescription.h"
#import "OEControllerDescription_Internal.h"

#import <IOKit/IOKitLib.h>
#import <IOKit/IOCFPlugIn.h>
#import <IOKit/usb/IOUSBLib.h>
#import <IOKit/hid/IOHIDUsageTables.h>
#import <signal.h>

NS_ASSUME_NONNULL_BEGIN

#pragma mark - GIP Protocol

// USB class codes of the GIP (Xbox One) interface, also used as device class codes.
#define GIP_CLASS    0xFF
#define GIP_SUBCLASS 0x47
#define GIP_PROTOCOL 0xD0

#define GIP_MAX_PACKET_SIZE 64
#define GIP_RECLAIM_INTERVAL_SECONDS 1.0
// System drivers attach to a device shortly after it appears; wait for them
// before deciding whether the device is ours to drive.
#define GIP_DRIVER_SETTLE_SECONDS 1.0

typedef NS_ENUM(uint8_t, OEGIPCommand) {
    OEGIPCommandAcknowledge = 0x01,
    OEGIPCommandPower       = 0x05,
    OEGIPCommandAuthenticate = 0x06,
    OEGIPCommandLED         = 0x0A,
    OEGIPCommandGuideButton = 0x07,
    OEGIPCommandInput       = 0x20,
};

#define GIP_FLAG_INTERNAL     0x20
#define GIP_FLAG_NEEDS_ACK    0x10

// Byte 4-5 of an input report, little-endian.
typedef NS_OPTIONS(uint16_t, OEGIPButtons) {
    OEGIPButtonMenu      = 1 << 2,
    OEGIPButtonView      = 1 << 3,
    OEGIPButtonA         = 1 << 4,
    OEGIPButtonB         = 1 << 5,
    OEGIPButtonX         = 1 << 6,
    OEGIPButtonY         = 1 << 7,
    OEGIPButtonDPadUp    = 1 << 8,
    OEGIPButtonDPadDown  = 1 << 9,
    OEGIPButtonDPadLeft  = 1 << 10,
    OEGIPButtonDPadRight = 1 << 11,
    OEGIPButtonLB        = 1 << 12,
    OEGIPButtonRB        = 1 << 13,
    OEGIPButtonLS        = 1 << 14,
    OEGIPButtonRS        = 1 << 15,
};

typedef struct __attribute__((packed)) {
    uint8_t command;
    uint8_t flags;
    uint8_t sequence;
    uint8_t length;
    uint16_t buttons;
    uint16_t leftTrigger;   // 0-1023
    uint16_t rightTrigger;  // 0-1023
    int16_t leftStickX;
    int16_t leftStickY;     // positive is up
    int16_t rightStickX;
    int16_t rightStickY;    // positive is up
} OEGIPInputReport;

#define GIP_TRIGGER_MAX 1023

// Right sticks of Xbox pads often rest well off center (15% on a PowerA pad),
// past OpenEmu's default dead zone, which would register as a held direction.
// Use Microsoft's recommended right stick dead zone (XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE).
// The left stick keeps the default, which feels noticeably more responsive.
#define GIP_RIGHT_STICK_DEAD_ZONE (8689.0 / INT16_MAX)

// Button numbers used by the OEControllerMicrosoftXboxOne controller description.
static const struct { OEGIPButtons mask; NSUInteger number; } OEGIPButtonMap[] = {
    { OEGIPButtonA,    1 },
    { OEGIPButtonB,    2 },
    { OEGIPButtonX,    4 },
    { OEGIPButtonY,    5 },
    { OEGIPButtonLB,   7 },
    { OEGIPButtonRB,   8 },
    { OEGIPButtonMenu, 12 },
    { OEGIPButtonLS,   14 },
    { OEGIPButtonRS,   15 },
    { OEGIPButtonView, 548 },
};
static const NSUInteger OEGIPGuideButtonNumber = 547;

// Every control's cookie is derived from its usage, which is unique within
// the description (buttons 1-15 and 547-548, axes 0x30-0x35, triggers 0xC4-0xC5).
static inline NSUInteger OEGIPCookieForUsage(NSUInteger usage)
{
    return usage + 1;
}

static NSString *const OEGIPClaimNotification = @"org.openemu.OpenEmuSystem.GIPControllerClaim";
static NSString *const OEGIPReleaseNotification = @"org.openemu.OpenEmuSystem.GIPControllerRelease";

static void OEGIPReadCompleted(void *refcon, IOReturn result, void *arg0);
static void OEGIPWriteCompleted(void *refcon, IOReturn result, void *arg0);
static void OEGIPDeviceMatched(void * _Nullable refcon, io_iterator_t iterator);
static void OEGIPAddDeviceIfAvailable(io_service_t service);
static void OEGIPDeviceInterest(void *refcon, io_service_t service, natural_t messageType, void *messageArgument);

#pragma mark - Device Handler

@implementation OEGIPUSBDeviceHandler
{
    io_service_t _service;
    IOUSBInterfaceInterface650 **_interface;
    CFRunLoopSourceRef _asyncSource;
    UInt8 _inPipe, _outPipe;
    uint8_t _readBuffer[GIP_MAX_PACKET_SIZE];
    uint8_t _sequence;

    NSString *_uniqueIdentifier;
    NSString *_product, *_manufacturer, *_serialNumber;
    NSNumber *_locationID;

    NSMutableDictionary<NSNumber *, OEHIDEvent *> *_latestEvents;
    io_object_t _interestNotification;
    BOOL _receivedInputSinceOpen;
    NSTimer *_reclaimTimer;
    // The other process that most recently claimed the controller, and when.
    pid_t _claimingPID;
    // When this process last claimed the controller, or 0 if it has no
    // outstanding claim. Claims are ordered by time: the most recent wins.
    CFAbsoluteTime _ownClaimTime;
}

#pragma mark Device monitoring

static IONotificationPortRef OEGIPNotificationPort;
static NSMutableDictionary<NSNumber *, OEGIPUSBDeviceHandler *> *OEGIPHandlersByEntryID;
static void (^OEGIPAddHandler)(OEGIPUSBDeviceHandler *);
static void (^OEGIPRemoveHandler)(OEGIPUSBDeviceHandler *);
static BOOL OEGIPWantsDevices;

+ (void)startMonitoringWithAddHandler:(void (^)(OEGIPUSBDeviceHandler *))addHandler removeHandler:(void (^)(OEGIPUSBDeviceHandler *))removeHandler
{
    NSAssert(NSThread.isMainThread, @"GIP device monitoring must start on the main thread");
    if (OEGIPNotificationPort != NULL)
        return;

    OEGIPAddHandler = [addHandler copy];
    OEGIPRemoveHandler = [removeHandler copy];
    OEGIPHandlersByEntryID = [NSMutableDictionary dictionary];

    OEGIPNotificationPort = IONotificationPortCreate(MACH_PORT_NULL);
    CFRunLoopAddSource(CFRunLoopGetMain(), IONotificationPortGetRunLoopSource(OEGIPNotificationPort), kCFRunLoopDefaultMode);

    NSDictionary *matching = @{
        @kIOProviderClassKey : @kIOUSBHostDeviceClassName,
        @kIOPropertyMatchKey : @{
            @kUSBDeviceClass    : @GIP_CLASS,
            @kUSBDeviceSubClass : @GIP_SUBCLASS,
            @kUSBDeviceProtocol : @GIP_PROTOCOL,
        },
    };

    io_iterator_t iterator;
    kern_return_t kr = IOServiceAddMatchingNotification(OEGIPNotificationPort, kIOFirstMatchNotification, (CFDictionaryRef)CFBridgingRetain(matching), OEGIPDeviceMatched, NULL, &iterator);
    if (kr != KERN_SUCCESS) {
        NSLog(@"GIP: unable to watch for USB devices (0x%08x)", kr);
        return;
    }
    // Arm the notification and pick up devices that are already connected.
    OEGIPDeviceMatched(NULL, iterator);
}

static void OEGIPDeviceMatched(void * _Nullable refcon, io_iterator_t iterator)
{
    io_service_t service;
    while ((service = IOIteratorNext(iterator))) {
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(GIP_DRIVER_SETTLE_SECONDS * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{
            OEGIPAddDeviceIfAvailable(service);
            IOObjectRelease(service);
        });
    }
}

/// Returns YES if a system driver, such as Apple's XboxGamepad driver for
/// official Microsoft controllers, already drives the device's interfaces.
/// Such devices are exposed through HID and need no handler of their own.
static BOOL OEGIPDeviceIsDrivenBySystem(io_service_t device)
{
    BOOL driven = NO;
    io_iterator_t interfaces;
    if (IORegistryEntryGetChildIterator(device, kIOServicePlane, &interfaces) != KERN_SUCCESS)
        return NO;

    io_service_t interface;
    while (!driven && (interface = IOIteratorNext(interfaces))) {
        io_iterator_t clients;
        if (IOObjectConformsTo(interface, "IOUSBHostInterface") &&
            IORegistryEntryGetChildIterator(interface, kIOServicePlane, &clients) == KERN_SUCCESS) {
            io_service_t client;
            while (!driven && (client = IOIteratorNext(clients))) {
                io_name_t className;
                // User clients belong to processes like this one; anything else is a driver.
                if (IOObjectGetClass(client, className) == KERN_SUCCESS && strstr(className, "UserClient") == NULL)
                    driven = YES;
                IOObjectRelease(client);
            }
            IOObjectRelease(clients);
        }
        IOObjectRelease(interface);
    }
    IOObjectRelease(interfaces);
    return driven;
}

/// Returns YES for devices that Apple's XboxGamepad driver matches. Its
/// personalities are read from the driver itself, so the list stays current.
static BOOL OEGIPDeviceIsSupportedBySystemDriver(NSUInteger vendorID, NSUInteger productID)
{
    static NSSet<NSNumber *> *supportedDevices;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        NSMutableSet<NSNumber *> *devices = [NSMutableSet set];
        NSDictionary *info = [NSDictionary dictionaryWithContentsOfFile:@"/System/Library/DriverExtensions/XboxGamepad.dext/Info.plist"];
        for (NSDictionary *personality in [info[@"IOKitPersonalities"] allValues]) {
            NSNumber *vid = personality[@kUSBVendorID], *pid = personality[@kUSBProductID];
            if ([vid isKindOfClass:NSNumber.class] && [pid isKindOfClass:NSNumber.class])
                [devices addObject:@((vid.unsignedIntegerValue << 16) | pid.unsignedIntegerValue)];
        }
        supportedDevices = [devices copy];
    });
    return [supportedDevices containsObject:@((vendorID << 16) | productID)];
}

static void OEGIPAddDeviceIfAvailable(io_service_t service)
{
    uint64_t entryID = 0;
    if (IORegistryEntryGetRegistryEntryID(service, &entryID) != KERN_SUCCESS || OEGIPHandlersByEntryID[@(entryID)] != nil)
        return;

    // The device may have been unplugged while waiting for drivers to settle.
    io_service_t current = IOServiceGetMatchingService(MACH_PORT_NULL, IORegistryEntryIDMatching(entryID));
    if (current == 0)
        return;
    IOObjectRelease(current);

    if (OEGIPDeviceIsDrivenBySystem(service))
        return;

    OEGIPUSBDeviceHandler *handler = [OEGIPUSBDeviceHandler OE_handlerWithService:service];
    if (handler == nil || OEGIPDeviceIsSupportedBySystemDriver(handler.vendorID, handler.productID))
        return;

    // Without a termination notice the handler would outlive the device.
    if (IOServiceAddInterestNotification(OEGIPNotificationPort, service, kIOGeneralInterest, OEGIPDeviceInterest, (void *)entryID, &handler->_interestNotification) != KERN_SUCCESS)
        return;

    OEGIPHandlersByEntryID[@(entryID)] = handler;
    if ([handler connect])
        OEGIPAddHandler(handler);
}

static void OEGIPDeviceInterest(void *refcon, io_service_t service, natural_t messageType, void *messageArgument)
{
    if (messageType != kIOMessageServiceIsTerminated)
        return;

    NSNumber *entryID = @((uint64_t)refcon);
    OEGIPUSBDeviceHandler *handler = OEGIPHandlersByEntryID[entryID];
    if (handler == nil)
        return;

    [OEGIPHandlersByEntryID removeObjectForKey:entryID];
    // The device manager disconnects the handler as part of removing it.
    OEGIPRemoveHandler(handler);
}

+ (void)setWantsDevices:(BOOL)wantsDevices
{
    if (OEGIPWantsDevices == wantsDevices)
        return;

    OEGIPWantsDevices = wantsDevices;
    for (OEGIPUSBDeviceHandler *handler in OEGIPHandlersByEntryID.allValues) {
        if (wantsDevices)
            [handler OE_claim];
        else
            [handler OE_relinquish];
    }
}

#pragma mark Initialization

+ (nullable instancetype)OE_handlerWithService:(io_service_t)service
{
    NSNumber *vendorID = CFBridgingRelease(IORegistryEntryCreateCFProperty(service, CFSTR(kUSBVendorID), kCFAllocatorDefault, 0));
    NSNumber *productID = CFBridgingRelease(IORegistryEntryCreateCFProperty(service, CFSTR(kUSBProductID), kCFAllocatorDefault, 0));
    NSString *product = CFBridgingRelease(IORegistryEntryCreateCFProperty(service, CFSTR(kUSBProductString), kCFAllocatorDefault, 0)) ?: @"Xbox Controller";
    if (vendorID == nil || productID == nil)
        return nil;

    // Reuse the description of the Xbox One controller so that GIP controllers
    // share its control names and bindings.
    OEControllerDescription *controllerDesc = [OEControllerDescription OE_controllerDescriptionForVendorID:0x045E productID:0x02FD product:@"Xbox One S Wireless"];
    if (![controllerDesc.identifier isEqualToString:@"OEControllerMicrosoftXboxOne"]) {
        NSLog(@"GIP: Xbox One controller description not found");
        return nil;
    }

    NSDictionary *representations = [OEControllerDescription OE_representationForControllerDescription:controllerDesc];
    [representations enumerateKeysAndObjectsUsingBlock:^(NSString *identifier, NSDictionary *representation, BOOL *stop) {
        OEHIDEventType type = OEHIDEventTypeFromNSString(representation[@"Type"]);
        NSUInteger usage = OEUsageFromUsageStringWithType(representation[@"Usage"], type);
        NSUInteger cookie = OEGIPCookieForUsage(usage);

        OEHIDEvent *event;
        switch (type) {
            case OEHIDEventTypeAxis:
                event = [OEHIDEvent axisEventWithDeviceHandler:nil timestamp:0 axis:usage direction:OEHIDEventAxisDirectionNull cookie:cookie];
                break;
            case OEHIDEventTypeTrigger:
                // Like OEHIDEvent's HID element events, a trigger control is described
                // by its pulled state, which is what the system responders look up.
                event = [OEHIDEvent triggerEventWithDeviceHandler:nil timestamp:0 axis:usage direction:OEHIDEventAxisDirectionPositive cookie:cookie];
                break;
            case OEHIDEventTypeButton:
                event = [OEHIDEvent buttonEventWithDeviceHandler:nil timestamp:0 buttonNumber:usage state:OEHIDEventStateOn cookie:cookie];
                break;
            case OEHIDEventTypeHatSwitch:
                event = [OEHIDEvent hatSwitchEventWithDeviceHandler:nil timestamp:0 type:OEHIDEventHatSwitchType8Ways direction:OEHIDEventHatDirectionNull cookie:cookie];
                break;
            default:
                NSLog(@"GIP: unexpected control type in Xbox One controller description");
                return;
        }

        [controllerDesc addControlWithIdentifier:identifier name:representation[@"Name"] event:event valueRepresentations:representation[@"Values"]];
    }];

    OEDeviceDescription *deviceDesc = [controllerDesc OE_addDeviceDescriptionWithVendorID:vendorID.unsignedIntegerValue productID:productID.unsignedIntegerValue product:product cookie:0];

    OEGIPUSBDeviceHandler *handler = [[self alloc] initWithDeviceDescription:deviceDesc];
    for (OEControlDescription *control in controllerDesc.controls) {
        OEHIDEventAxis axis = control.type == OEHIDEventTypeAxis ? control.genericEvent.axis : OEHIDEventAxisNone;
        if (axis == OEHIDEventAxisZ || axis == OEHIDEventAxisRz)
            [handler setDeadZone:GIP_RIGHT_STICK_DEAD_ZONE forControlDescription:control];
    }

    IOObjectRetain(service);
    handler->_service = service;
    handler->_product = product;
    handler->_manufacturer = CFBridgingRelease(IORegistryEntryCreateCFProperty(service, CFSTR(kUSBVendorString), kCFAllocatorDefault, 0));
    handler->_serialNumber = CFBridgingRelease(IORegistryEntryCreateCFProperty(service, CFSTR(kUSBSerialNumberString), kCFAllocatorDefault, 0));
    handler->_locationID = CFBridgingRelease(IORegistryEntryCreateCFProperty(service, CFSTR(kUSBDevicePropertyLocationID), kCFAllocatorDefault, 0));
    return handler;
}

- (instancetype)initWithDeviceDescription:(nullable OEDeviceDescription *)deviceDescription
{
    if ((self = [super initWithDeviceDescription:deviceDescription])) {
        _latestEvents = [NSMutableDictionary dictionary];
    }
    return self;
}

- (void)dealloc
{
    [self OE_closeInterface];
    if (_interestNotification)
        IOObjectRelease(_interestNotification);
    if (_service)
        IOObjectRelease(_service);
}

#pragma mark Properties

- (NSString *)uniqueIdentifier
{
    if (_uniqueIdentifier == nil)
        _uniqueIdentifier = [NSString stringWithFormat:@"GIP_%@", _locationID ?: _serialNumber ?: _product];
    return _uniqueIdentifier;
}

- (NSString *)serialNumber { return _serialNumber ?: @""; }
- (NSString *)manufacturer { return _manufacturer ?: @""; }
- (NSString *)product      { return _product; }
- (NSNumber *)locationID   { return _locationID; }

#pragma mark Connection and ownership

- (BOOL)connect
{
    NSDistributedNotificationCenter *center = [NSDistributedNotificationCenter defaultCenter];
    [center addObserver:self selector:@selector(OE_otherProcessDidClaim:) name:OEGIPClaimNotification object:nil suspensionBehavior:NSNotificationSuspensionBehaviorDeliverImmediately];
    [center addObserver:self selector:@selector(OE_otherProcessDidRelease:) name:OEGIPReleaseNotification object:nil suspensionBehavior:NSNotificationSuspensionBehaviorDeliverImmediately];

    if (OEGIPWantsDevices)
        [self OE_claim];

    __weak typeof(self) weakSelf = self;
    _reclaimTimer = [NSTimer timerWithTimeInterval:GIP_RECLAIM_INTERVAL_SECONDS repeats:YES block:^(NSTimer *timer) {
        [weakSelf OE_reclaimIfPossible];
    }];
    [[NSRunLoop mainRunLoop] addTimer:_reclaimTimer forMode:NSRunLoopCommonModes];

    return YES;
}

- (void)disconnect
{
    [_reclaimTimer invalidate];
    _reclaimTimer = nil;
    [[NSDistributedNotificationCenter defaultCenter] removeObserver:self];
    [self OE_closeInterface];
}

/// Asks any other OpenEmu process holding this controller to let go of it.
- (void)OE_claim
{
    _claimingPID = 0;
    _ownClaimTime = CFAbsoluteTimeGetCurrent();
    [self OE_postNotificationNamed:OEGIPClaimNotification time:_ownClaimTime];

    // The current owner closes the interface when it receives the claim;
    // until then opening fails, and the reclaim timer retries.
    if (![self OE_openInterface]) {
        __weak typeof(self) weakSelf = self;
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(0.1 * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{
            [weakSelf OE_reclaimIfPossible];
        });
    }
}

/// Lets a process that is waiting for this controller take it back.
- (void)OE_relinquish
{
    // A claim that never got the interface still made the other process
    // yield, so it must be released as well.
    if (_interface == NULL && _ownClaimTime == 0)
        return;

    if (_interface != NULL)
        NSLog(@"GIP: releasing %@", _product);
    _ownClaimTime = 0;
    [self OE_releaseAllControls];
    [self OE_closeInterface];
    [self OE_postNotificationNamed:OEGIPReleaseNotification time:CFAbsoluteTimeGetCurrent()];
}

// Distributed notifications can't carry a userInfo from sandboxed senders,
// so the controller, the sending process and the time are encoded in the object.
- (void)OE_postNotificationNamed:(NSString *)name time:(CFAbsoluteTime)time
{
    NSString *object = [NSString stringWithFormat:@"%@|%d|%f", self.uniqueIdentifier, getpid(), time];
    [[NSDistributedNotificationCenter defaultCenter] postNotificationName:name object:object userInfo:nil deliverImmediately:YES];
}

/// Returns the sending process of a notification about this controller, or 0
/// if it is about another controller or was sent by this process.
- (pid_t)OE_otherProcessOfNotification:(NSNotification *)notification time:(nullable CFAbsoluteTime *)outTime
{
    NSArray<NSString *> *parts = [(NSString *)notification.object componentsSeparatedByString:@"|"];
    if (parts.count != 3 || ![parts[0] isEqualToString:self.uniqueIdentifier])
        return 0;

    pid_t pid = parts[1].intValue;
    if (outTime)
        *outTime = parts[2].doubleValue;
    return pid == getpid() ? 0 : pid;
}

- (void)OE_otherProcessDidClaim:(NSNotification *)notification
{
    CFAbsoluteTime claimTime;
    pid_t pid = [self OE_otherProcessOfNotification:notification time:&claimTime];
    if (pid == 0)
        return;

    // When two claims cross, both processes see both of them; the later claim
    // wins, so exactly one process yields. Ties go to the higher process ID.
    if (_ownClaimTime > claimTime || (_ownClaimTime == claimTime && getpid() > pid))
        return;

    if (_interface != NULL)
        NSLog(@"GIP: %@ claimed by process %d", _product, pid);
    _claimingPID = pid;
    _ownClaimTime = 0;
    [self OE_releaseAllControls];
    [self OE_closeInterface];
}

- (void)OE_otherProcessDidRelease:(NSNotification *)notification
{
    pid_t pid = [self OE_otherProcessOfNotification:notification time:NULL];
    if (pid == 0 || pid != _claimingPID)
        return;

    _claimingPID = 0;
    [self OE_reclaimIfPossible];
}

- (void)OE_reclaimIfPossible
{
    if (!OEGIPWantsDevices || _interface != NULL)
        return;

    // Wait until the process that claimed the controller releases it or exits.
    if (_claimingPID != 0 && kill(_claimingPID, 0) == 0)
        return;

    if ([self OE_openInterface])
        _claimingPID = 0;
}

#pragma mark USB

- (BOOL)OE_openInterface
{
    if (_interface != NULL)
        return YES;

    // A system driver may have attached after the device was first seen.
    if (OEGIPDeviceIsDrivenBySystem(_service) || ![self OE_configureDeviceIfNeeded])
        return NO;

    IOUSBInterfaceInterface650 **interface = [self OE_createGIPInterface];
    if (interface == NULL)
        return NO;

    IOReturn kr = (*interface)->USBInterfaceOpen(interface);
    if (kr != kIOReturnSuccess) {
        // kIOReturnExclusiveAccess: another process, or a system driver, owns it.
        if (kr != kIOReturnExclusiveAccess)
            NSLog(@"GIP: unable to open interface of %@ (0x%08x)", _product, kr);
        (*interface)->Release(interface);
        return NO;
    }

    _inPipe = _outPipe = 0;
    UInt8 numEndpoints = 0;
    (*interface)->GetNumEndpoints(interface, &numEndpoints);
    for (UInt8 pipe = 1; pipe <= numEndpoints; pipe++) {
        UInt8 direction, number, transferType, interval;
        UInt16 maxPacketSize;
        if ((*interface)->GetPipeProperties(interface, pipe, &direction, &number, &transferType, &maxPacketSize, &interval) != kIOReturnSuccess)
            continue;
        if (transferType != kUSBInterrupt)
            continue;
        if (direction == kUSBIn && _inPipe == 0)
            _inPipe = pipe;
        else if (direction == kUSBOut && _outPipe == 0)
            _outPipe = pipe;
    }

    if (_inPipe == 0 || _outPipe == 0 ||
        (*interface)->CreateInterfaceAsyncEventSource(interface, &_asyncSource) != kIOReturnSuccess) {
        NSLog(@"GIP: %@ has no usable interrupt pipes", _product);
        (*interface)->USBInterfaceClose(interface);
        (*interface)->Release(interface);
        return NO;
    }

    _interface = interface;
    _receivedInputSinceOpen = NO;
    CFRunLoopAddSource(CFRunLoopGetMain(), _asyncSource, kCFRunLoopCommonModes);

    if (![self OE_scheduleRead]) {
        [self OE_closeInterface];
        return NO;
    }

    [self OE_sendStartupPackets];
    NSLog(@"GIP: connected to %@", _product);
    return YES;
}

/// Selects the device's first configuration if no driver has done it yet.
/// Interfaces, including the GIP one, only exist once a configuration is set.
- (BOOL)OE_configureDeviceIfNeeded
{
    IOCFPlugInInterface **plugin = NULL;
    SInt32 score;
    if (IOCreatePlugInInterfaceForService(_service, kIOUSBDeviceUserClientTypeID, kIOCFPlugInInterfaceID, &plugin, &score) != kIOReturnSuccess || plugin == NULL)
        return NO;

    IOUSBDeviceInterface650 **device = NULL;
    (*plugin)->QueryInterface(plugin, CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID650), (LPVOID *)&device);
    IODestroyPlugInInterface(plugin);
    if (device == NULL)
        return NO;

    BOOL success = YES;
    UInt8 currentConfig = 0;
    (*device)->GetConfiguration(device, &currentConfig);
    if (currentConfig == 0) {
        IOUSBConfigurationDescriptorPtr config;
        IOReturn kr = (*device)->USBDeviceOpen(device);
        if (kr == kIOReturnSuccess) {
            kr = (*device)->GetConfigurationDescriptorPtr(device, 0, &config);
            if (kr == kIOReturnSuccess)
                kr = (*device)->SetConfiguration(device, config->bConfigurationValue);
            (*device)->USBDeviceClose(device);
        }
        if (kr != kIOReturnSuccess) {
            if (kr != kIOReturnExclusiveAccess)
                NSLog(@"GIP: unable to configure %@ (0x%08x)", _product, kr);
            success = NO;
        }
    }

    (*device)->Release(device);
    return success;
}

- (IOUSBInterfaceInterface650 **)OE_createGIPInterface
{
    IOCFPlugInInterface **plugin = NULL;
    SInt32 score;
    if (IOCreatePlugInInterfaceForService(_service, kIOUSBDeviceUserClientTypeID, kIOCFPlugInInterfaceID, &plugin, &score) != kIOReturnSuccess || plugin == NULL)
        return NULL;

    IOUSBDeviceInterface650 **device = NULL;
    (*plugin)->QueryInterface(plugin, CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID650), (LPVOID *)&device);
    IODestroyPlugInInterface(plugin);
    if (device == NULL)
        return NULL;

    IOUSBFindInterfaceRequest request = {
        .bInterfaceClass    = GIP_CLASS,
        .bInterfaceSubClass = GIP_SUBCLASS,
        .bInterfaceProtocol = GIP_PROTOCOL,
        .bAlternateSetting  = kIOUSBFindInterfaceDontCare,
    };
    io_iterator_t iterator;
    io_service_t interfaceService = 0;
    if ((*device)->CreateInterfaceIterator(device, &request, &iterator) == kIOReturnSuccess) {
        // The first GIP interface carries input and output; later ones are for audio.
        interfaceService = IOIteratorNext(iterator);
        IOObjectRelease(iterator);
    }
    (*device)->Release(device);
    if (interfaceService == 0)
        return NULL;

    IOUSBInterfaceInterface650 **interface = NULL;
    if (IOCreatePlugInInterfaceForService(interfaceService, kIOUSBInterfaceUserClientTypeID, kIOCFPlugInInterfaceID, &plugin, &score) == kIOReturnSuccess && plugin != NULL) {
        (*plugin)->QueryInterface(plugin, CFUUIDGetUUIDBytes(kIOUSBInterfaceInterfaceID650), (LPVOID *)&interface);
        IODestroyPlugInInterface(plugin);
    }
    IOObjectRelease(interfaceService);
    return interface;
}

- (void)OE_closeInterface
{
    if (_interface == NULL)
        return;

    if (_asyncSource) {
        CFRunLoopRemoveSource(CFRunLoopGetMain(), _asyncSource, kCFRunLoopCommonModes);
        CFRelease(_asyncSource);
        _asyncSource = NULL;
    }
    // Closing aborts the pending read; its callback sees kIOReturnAborted.
    (*_interface)->USBInterfaceClose(_interface);
    (*_interface)->Release(_interface);
    _interface = NULL;
}

- (BOOL)OE_scheduleRead
{
    // ReadPipeTO/WritePipeTO reject interrupt pipes, so reads are asynchronous.
    IOReturn kr = (*_interface)->ReadPipeAsync(_interface, _inPipe, _readBuffer, sizeof(_readBuffer), OEGIPReadCompleted, (__bridge void *)self);
    if (kr != kIOReturnSuccess) {
        NSLog(@"GIP: unable to read from %@ (0x%08x)", _product, kr);
        return NO;
    }
    return YES;
}

- (void)OE_readCompletedWithResult:(IOReturn)result length:(UInt32)length
{
    if (_interface == NULL || result == kIOReturnAborted)
        return;

    if (result == kIOReturnSuccess)
        [self OE_handlePacket:_readBuffer length:length];
    else if (result == kIOReturnNoDevice || result == kIOReturnNotResponding) {
        // Let go of the interface; the reclaim timer reopens it if the device
        // recovers, and the termination notice removes it if it is gone.
        NSLog(@"GIP: %@ stopped responding (0x%08x)", _product, result);
        [self OE_releaseAllControls];
        [self OE_closeInterface];
        return;
    } else {
        NSLog(@"GIP: read from %@ failed (0x%08x), clearing stall", _product, result);
        (*_interface)->ClearPipeStallBothEnds(_interface, _inPipe);
    }

    // Dispatching events may have led to the interface being closed.
    if (_interface != NULL && ![self OE_scheduleRead]) {
        [self OE_releaseAllControls];
        [self OE_closeInterface];
    }
}

- (void)OE_sendPacket:(uint8_t *)packet length:(UInt32)length
{
    packet[2] = _sequence++;
    [self OE_writePacket:packet length:length];
}

- (void)OE_writePacket:(uint8_t *)packet length:(UInt32)length
{
    if (_interface == NULL)
        return;

    // Writes are asynchronous so that a pad that stops accepting data can't
    // block the main thread; the buffer is freed when the write completes.
    void *buffer = malloc(length);
    memcpy(buffer, packet, length);
    IOReturn kr = (*_interface)->WritePipeAsync(_interface, _outPipe, buffer, length, OEGIPWriteCompleted, buffer);
    if (kr != kIOReturnSuccess) {
        NSLog(@"GIP: unable to write to %@ (0x%08x)", _product, kr);
        free(buffer);
    }
}

- (void)OE_sendStartupPackets
{
    // The sequence the Linux xpad driver sends to Xbox One-family pads: power
    // on, the extended init that Microsoft's Xbox One S and Elite 2 pads need,
    // turn the Xbox button LED on, then report authentication as done, which
    // some third-party pads wait for before they send input.
    uint8_t powerOn[]  = { OEGIPCommandPower,        GIP_FLAG_INTERNAL, 0, 0x01, 0x00 };
    uint8_t sInit[]    = { OEGIPCommandPower,        GIP_FLAG_INTERNAL, 0, 0x0F, 0x06 };
    uint8_t ledOn[]    = { OEGIPCommandLED,          GIP_FLAG_INTERNAL, 0, 0x03, 0x00, 0x01, 0x14 };
    uint8_t authDone[] = { OEGIPCommandAuthenticate, GIP_FLAG_INTERNAL, 0, 0x02, 0x01, 0x00 };
    [self OE_sendPacket:powerOn length:sizeof(powerOn)];
    if (self.vendorID == 0x045E && (self.productID == 0x02EA || self.productID == 0x0B00))
        [self OE_sendPacket:sInit length:sizeof(sInit)];
    [self OE_sendPacket:ledOn length:sizeof(ledOn)];
    [self OE_sendPacket:authDone length:sizeof(authDone)];
}

#pragma mark Input

- (void)OE_handlePacket:(const uint8_t *)packet length:(UInt32)length
{
    if (length < 4)
        return;

    switch (packet[0]) {
        case OEGIPCommandInput:
            if (!_receivedInputSinceOpen) {
                _receivedInputSinceOpen = YES;
                NSLog(@"GIP: receiving input from %@", _product);
            }
            if (length >= sizeof(OEGIPInputReport))
                [self OE_dispatchEventsWithInputReport:(const OEGIPInputReport *)packet];
            break;

        case OEGIPCommandGuideButton:
            if (length < 5)
                break;
            if (packet[1] & GIP_FLAG_NEEDS_ACK) {
                // The ack echoes the sequence number of the packet it acknowledges.
                uint8_t ack[] = { OEGIPCommandAcknowledge, GIP_FLAG_INTERNAL, packet[2], 0x09, 0x00,
                                  OEGIPCommandGuideButton, GIP_FLAG_INTERNAL, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
                [self OE_writePacket:ack length:sizeof(ack)];
            }
            [self OE_dispatchButton:OEGIPGuideButtonNumber pressed:(packet[4] & 0x01) timestamp:NSDate.timeIntervalSinceReferenceDate];
            break;

        default:
            // Status heartbeats (0x03), announcements (0x02) and others carry no input.
            break;
    }
}

- (void)OE_dispatchEventsWithInputReport:(const OEGIPInputReport *)report
{
    NSTimeInterval now = NSDate.timeIntervalSinceReferenceDate;
    uint16_t buttons = OSSwapLittleToHostInt16(report->buttons);

    for (size_t i = 0; i < sizeof(OEGIPButtonMap) / sizeof(OEGIPButtonMap[0]); i++)
        [self OE_dispatchButton:OEGIPButtonMap[i].number pressed:(buttons & OEGIPButtonMap[i].mask) != 0 timestamp:now];

    OEHIDEventHatDirection hat = OEHIDEventHatDirectionNull;
    if (buttons & OEGIPButtonDPadUp)    hat |= OEHIDEventHatDirectionNorth;
    if (buttons & OEGIPButtonDPadDown)  hat |= OEHIDEventHatDirectionSouth;
    if (buttons & OEGIPButtonDPadLeft)  hat |= OEHIDEventHatDirectionWest;
    if (buttons & OEGIPButtonDPadRight) hat |= OEHIDEventHatDirectionEast;
    [self OE_dispatchEvent:[OEHIDEvent hatSwitchEventWithDeviceHandler:self timestamp:now type:OEHIDEventHatSwitchType8Ways direction:hat cookie:OEGIPCookieForUsage(kHIDUsage_GD_Hatswitch)]];

    // The description maps the right stick to Z/Rz. GIP reports Y as positive-up,
    // while OpenEmu expects positive-down, as HID gamepads report it.
    [self OE_dispatchAxis:OEHIDEventAxisX  rawValue:(int16_t)OSSwapLittleToHostInt16(report->leftStickX)   inverted:NO  timestamp:now];
    [self OE_dispatchAxis:OEHIDEventAxisY  rawValue:(int16_t)OSSwapLittleToHostInt16(report->leftStickY)   inverted:YES timestamp:now];
    [self OE_dispatchAxis:OEHIDEventAxisZ  rawValue:(int16_t)OSSwapLittleToHostInt16(report->rightStickX)  inverted:NO  timestamp:now];
    [self OE_dispatchAxis:OEHIDEventAxisRz rawValue:(int16_t)OSSwapLittleToHostInt16(report->rightStickY)  inverted:YES timestamp:now];

    [self OE_dispatchTrigger:OEHIDEventAxisBrake       rawValue:OSSwapLittleToHostInt16(report->leftTrigger)  timestamp:now];
    [self OE_dispatchTrigger:OEHIDEventAxisAccelerator rawValue:OSSwapLittleToHostInt16(report->rightTrigger) timestamp:now];
}

- (void)OE_dispatchButton:(NSUInteger)number pressed:(BOOL)pressed timestamp:(NSTimeInterval)timestamp
{
    [self OE_dispatchEvent:[OEHIDEvent buttonEventWithDeviceHandler:self timestamp:timestamp buttonNumber:number state:pressed ? OEHIDEventStateOn : OEHIDEventStateOff cookie:OEGIPCookieForUsage(number)]];
}

- (void)OE_dispatchAxis:(OEHIDEventAxis)axis rawValue:(int16_t)rawValue inverted:(BOOL)inverted timestamp:(NSTimeInterval)timestamp
{
    NSUInteger cookie = OEGIPCookieForUsage(axis);
    CGFloat value = rawValue / (CGFloat)INT16_MAX;
    if (inverted)
        value = -value;
    if (fabs(value) < [self deadZoneForControlCookie:cookie])
        value = 0;
    [self OE_dispatchEvent:[OEHIDEvent axisEventWithDeviceHandler:self timestamp:timestamp axis:axis value:value cookie:cookie]];
}

- (void)OE_dispatchTrigger:(OEHIDEventAxis)axis rawValue:(uint16_t)rawValue timestamp:(NSTimeInterval)timestamp
{
    NSUInteger cookie = OEGIPCookieForUsage(axis);
    NSInteger value = MIN(rawValue, GIP_TRIGGER_MAX);
    if (value / (CGFloat)GIP_TRIGGER_MAX < [self deadZoneForControlCookie:cookie])
        value = 0;
    [self OE_dispatchEvent:[OEHIDEvent triggerEventWithDeviceHandler:self timestamp:timestamp axis:axis value:value maximum:GIP_TRIGGER_MAX cookie:cookie]];
}

/// Forwards events whose state changed, like OEHIDDeviceHandler does.
- (void)OE_dispatchEvent:(OEHIDEvent *)event
{
    NSNumber *cookieKey = @(event.cookie);
    OEHIDEvent *existingEvent = _latestEvents[cookieKey];
    if ([event isEqualToEvent:existingEvent])
        return;

    if ([event isAxisDirectionOppositeToEvent:existingEvent])
        [[OEDeviceManager sharedDeviceManager] deviceHandler:self didReceiveEvent:[event axisEventWithDirection:OEHIDEventAxisDirectionNull]];

    _latestEvents[cookieKey] = event;
    [[OEDeviceManager sharedDeviceManager] deviceHandler:self didReceiveEvent:event];
}

/// Sends a neutral event for every control that is currently active, so nothing
/// stays held down when another process takes over the controller.
- (void)OE_releaseAllControls
{
    for (OEHIDEvent *event in [_latestEvents.allValues copy]) {
        if (event.type == OEHIDEventTypeButton && event.state == OEHIDEventStateOn)
            [self OE_dispatchEvent:[OEHIDEvent buttonEventWithDeviceHandler:self timestamp:event.timestamp buttonNumber:event.buttonNumber state:OEHIDEventStateOff cookie:event.cookie]];
        else if (event.type == OEHIDEventTypeAxis || event.type == OEHIDEventTypeTrigger)
            // -nullEvent keeps the value, which analog keys would still read.
            [self OE_dispatchEvent:[event axisEventWithDirection:OEHIDEventAxisDirectionNull]];
        else if (event.type == OEHIDEventTypeHatSwitch)
            [self OE_dispatchEvent:[event nullEvent]];
    }
}

@end

static void OEGIPWriteCompleted(void *refcon, IOReturn result, void *arg0)
{
    free(refcon);
}

static void OEGIPReadCompleted(void *refcon, IOReturn result, void *arg0)
{
    OEGIPUSBDeviceHandler *handler = (__bridge OEGIPUSBDeviceHandler *)refcon;
    [handler OE_readCompletedWithResult:result length:(UInt32)(uintptr_t)arg0];
}

NS_ASSUME_NONNULL_END
