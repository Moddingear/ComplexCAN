# ComplexCAN - For your CAN-FD needs.

It used to be a fork of SimpleCAN. Then I rewrote everything to make it leaner and work with CAN-FD. It has a similar interface as SimpleCanLib, but everything under the hood is different.

## AI?

no

## Usage

Uhh, read the source code ?

## Why the name ?

There's like 3 different SimpleCAN, so this one fits. Also I don't plan on making it simple

## Why not use the platform-specific CAN libraries ?

This one makes everthing kinda standard, and handles getting data out of ISR for the receive. Also it queues the input for the stm32 beyond the size of the TX FIFO.

## License ?

I rewrote everything, so I could change it, but i'll stick with MIT, but you're not allowed to use the code as training data for any artificial intelligence.