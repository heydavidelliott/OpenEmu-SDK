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

#import <OpenEmuSystem/OEDeviceHandler.h>

NS_ASSUME_NONNULL_BEGIN

/// Handles wired Xbox One / Series controllers (GIP protocol) that macOS
/// does not expose as HID devices, e.g. third-party pads from PowerA, PDP,
/// Hori and others, which Apple's XboxGamepad driver does not match.
///
/// The controller is driven directly over its vendor-specific USB interface.
/// Only one process can own that interface at a time. Processes that need input
/// claim it from each other, and a process that loses it takes it back once the
/// claiming process releases it or exits.
@interface OEGIPUSBDeviceHandler : OEDeviceHandler

/// Starts watching for GIP USB devices; the blocks are called on the main thread.
+ (void)startMonitoringWithAddHandler:(void (^)(OEGIPUSBDeviceHandler *handler))addHandler
                        removeHandler:(void (^)(OEGIPUSBDeviceHandler *handler))removeHandler;

/// Whether this process needs live input from GIP controllers: while a game
/// runs, or while bindings are being recorded. Only one process can read a
/// controller at a time, so a process that doesn't need input lets go of it,
/// leaving it available to other OpenEmu processes and to other apps.
+ (void)setWantsDevices:(BOOL)wantsDevices;

@end

NS_ASSUME_NONNULL_END
