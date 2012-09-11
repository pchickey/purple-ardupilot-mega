Overview
========

AP\_HAL is hardware abstraction layer for the ArduPilot project. The goal is
to separate AVR (via avr-libc) and Arduino (via the Arduino core and included
libraries like SPI and Wire) dependencies from the ArduPilot programs and
libraries.

This will make it possible to port the ArduPlane and ArduCopter programs
(and their associated libraries) to a new platform without changing any of
the ArduPlane or ArduCopter code, only implementing the AP\_HAL interface
for the new platform.

Currently, the AP\_HAL\_AVR library , found in /libraries/AP\_HAL\_AVR/, is an
implementation of AP\_HAL for the ArduPilot hardware platforms.

AP\_HAL\_AVR exports two HAL instances, `AP_HAL_AVR_APM1` and `AP_HAL_AVR_APM2`,
which provide the 

Requirments
===========

AP\_HAL is designed to work with the Arduino 1.0.1 IDE with a special "Coreless"
modification, or with the ArduPilot Arduino.mk makefiles with a similar Coreless
extension. The Arduino.mk Makefile in the AP\_HAL development branch also
implements the coreless modification. You will not need to upgrade to a 
coreless patched IDE if you are just using Makefiles.

The Coreless modification to the Arduino IDE will be made available as a patch
and as compiled IDEs on the ArduPilot project's Downloads section.

Why Coreless Arduino
--------------------

todo

Consequences of Coreless Arduino
--------------------------------

* Need to provide an empty "Arduino.h" somewhere in the include path of each
  sketch. The line `#include <Arduino.h>` is added to each sketch by the
  Arduino IDE's sketch preprocessor, as well as by the Makefile build.

* Need to provide an entry point `int main(void)` for the application.
  Previously, a trivial implementation existed in the Arduino core:
    ```
    int main(void) {
        init(); /* Initialized the Arduino core functionality */
        setup();
        while(1) loop();
        return 0; /* Never reached */
    }
      
    ```

  Each program built with the coreless Arduino build will need to provide its
  own `main` function. The following implementation may be used in at the bottom
  of the sketch PDE/INO file:
    ```
    extern "C" {
        int main(void) {
            hal.init(NULL);
            setup();
            while(1) loop();
            return 0;
        }
    }
    ```
    The `extern "C"` wrapper is required because a PDE/INO file is compiled
    as C++.

* Global objects which were previously available in all Arduino sketches, such
  as Serial, Serial1, etc. no longer exist. This is by design - all of those
  hardware interfaces should be accessed through the HAL.


Using The AP\_HAL Library
=========================

AP\_HAL, found in /libraries/AP\_HAL/, is a library of purely virtual classes:
there is no concrete code in the AP\_HAL library, only interfaces. All code in
the core ArduPlane & ArduCopter programs, ArduPilot libraries, and example
sketches should depend only on the interfaces exposed by AP\_HAL.

The collection of classes in the AP\_HAL library exist in the AP\_HAL C++ 
namespace. The convention is for a program to instantiate a single instance
of the `AP_HAL::HAL` class, under a reference to the name `hal`.
    ```
    #include <AP_HAL.h>
    const AP_HAL::HAL& hal = specific_hal_implementation;
    ```
This instance should be made in a single object file.  All other object files,
including libraries (even those inside an AP\_HAL implementation, should use
the AP\_HAL interface by declaring an extern reference to `hal`.
    ```
    #include <AP_HAL.h>
    extern const AP_HAL::HAL& hal;
    ```


Using The AP\_HAL\AVR library
=============================

The AP\_HAL\_AVR library exports AP\_HAL::HAL instances for the APM1 and APM2. 
These instances are made of a number of concrete classes which implement the
AP\_HAL interfaces for the AVR platform. These implementations depend only on
avr-libc and not the Arduino core.

Some of the code in AP\_HAL\_AVR, such as the the GPIO class's pinMode, read,
and write, has been derived directly from the source code of the Arduino core
pinMode, digitalRead, and digitalWrite. 

When using the coreless Arduino IDE to build for AVR, you will need the
following three libraries included in the top level of your sketch:
    ```
    #include <AP_Common.h>
    #include <AP_HAL.h>
    #include <AP_HAL_AVR.h>
    ```
and then declare one of the following hal lines depending on your platform:
    ```
    const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
    ```
    or
    ```
    const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
    ```

AP\_HAL Library Contents
========================

The `AP_HAL` library is organized as follows:

AP\_HAL.h : exports all other headers for the library.

AP\_HAL\_Namespace.h : exposes the C++ namespace `AP_HAL`. The namespace
declaration declares each class by name (not implementation) and some useful
typedefs.

The AP\_HAL interface classes are each defined in a header file bearing their
name.


AP\_HAL::HAL
------------

The `AP_HAL::HAL` class is a container for the a complete set of device
drivers.  The class is defined in `/libraries/AP_HAL/HAL.h`.  It also has a
virtual (i.e. overridable) method to handle driver initialization.  Each device
driver is exposed as a pointer an AP\_HAL driver class, (e.g.  each serial
driver is exposed as a public `UARTDriver* uartN`).  

The following drivers are public members of `AP_HAL::HAL`. (Each class is in
the `AP_HAL` namespace, left off for brevity.)
    * `UARTDriver* uart0` : Corresponds to Arduino core Serial object
    * `UARTDriver* uart1` : Corresponds to Arduino core Serial1 object
    * `UARTDriver* uart2` : Corresponds to Arduino core Serial2 object
    * `UARTDriver* uart3` : Corresponds to Arduino core Serial3 object
    * `I2CDriver*  i2c` : Corresponds to ArduPilot `/libraries/I2C` driver
    * `SPIDriver*  spi` : Corresponds to Arduino library SPI object
    * `AnalogIn*   analogin` : Corresponds to
        `/libraries/AP_AnalogSource/AP_AnalogSource_Arduino` driver
    * `Storage*    storage` : Corresponds to avr-libc's `<avr/eeprom.h>`
        driver
    * `Dataflash*  dataflash` : Corresponds to ArduPilot `/libraries/DataFlash`
        driver
    * `BetterStream* console` : New utility for warning and error reporting
    * `GPIO*       gpio` : Corresponds to Arduino core `pinMode`, `digitalRead`,
        and `digitalWrite` functionality
    * `RCInput*    rcin` : Corresponds to PPM input side of ArduPilot
        `/libraries/APM_RC` library
    * `RCOutput*   rcout` : Corresponds to PPM output side of ArduPilot
        `/libraries/APM_RC` library
    * `Scheduler*  scheduler` : Encompasses both Arduino core timing functions
        such as `millis` and `delay` and the ArduPilot
        `/library/AP_PeriodicProcess/` driver.

`AP_HAL` also has an unimplemented `virtual void init(void* opts) const` method.
This method should initialize each driver before the call to a sketch's `setup`
method.



AP\_HAL::UARTDriver
--------------------

The `AP_HAL::UARTDriver` class is the AP\_HAL replacment for ArduPilot's
`FastSerial` library. 

The `AP_HAL::UARTDriver` class is a pure virtual interface. The code is derived
directly from the `FastSerial` class. It provides the methods `begin()`,
`end()`, `flush()`, `is_initialized()`, `set_blocking_writes`, and
`tx_pending`.  The class hierchary for `AP_HAL::UARTDriver` is also derived
directly from the `FastSerial` class's hierarchy . `AP_HAL::UARTDriver` is a
`public AP_HAL::BetterStream`, which is a `public AP_HAL::Stream`, which is a
`public AP_HAL::Print`.

The `utility/` directory contains the definitions of the `AP_HAL::Print`,
`AP_HAL::Stream`, and `AP_HAL::BetterStream` classes, as well as default
implementations for the `AP_HAL::Print` class.

`AP_HAL::Print` and `AP_HAL::Stream` are derived directly from the classes of
the same name in the Arduino core. Some methods dealing with the Arduino core's
C++ `String`s, and the `Printable` class, have been left out.

The `AP_HAL::Print` class has default implementations of all of the print
methods. Each of these methods is implemented in terms of
`virtual size_t write(uint8_t char)`, which is expected to be implemented by
a class deriving from `AP_HAL::Print`. This is the same structure as used in
the Arduino core.

The `AP_HAL::Stream` class is based on the Arduino core's Stream, but reduced
to just the methods we actually use in ArduPilot - `int available()`,
`int read()`, and `int peek()`. These methods are all pure virtual

The `AP_HAL::BetterStream` class is a pure virtual version of the class by the
same name from Mike Smith's `FastSerial` library. It exposes the methods
`int txspace()`, `void print_P()`, `void println_P`, `void printf()`, and
`void printf_P()`.  As in FastSerial's BetterStream library, function names
postfixed with \_P take a string in program space as their first argument.
To use the AVR program space types, the `AP_HAL::BetterStream` class depends on 
the `<avr/pgmspace.h>` and `AP_Common.h` header files. This is the only part of
the `AP_HAL` library which still depends on an AVR specific library. We will
find a way to make this dependency modular by isolating the parts of the 
`AP_Common` and `<avr/pgmspace.h>` headers which BetterStream depends upon
into a single header, and then conditionally defining those typedefs and
macros to innocuous implementations when compiling for other platforms.


AP\_HAL





Remaining ArduPilot AVR dependencies
====================================

The class `AP_Param` makes extensive use of AVR program space pointers and
accessors in its implementation. The definition of these types and accrssors
will have to be defined in a single external header file so they can be
replaced by innocuous types and accessors when compiling for other platforms


