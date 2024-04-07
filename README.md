#### Raspberry Pi Pico I2C Miner

A short program that takes the genesis block of Bitcoin, extracts the header, and attempts to find valid nonces that satisfy the target of the header.
Each block hash is sent to the connected I2C LCD display and shown. After each hash, the on board LED blinks to prove it is working. I used this program to power a breadboard desk piece.

Built with `cargo generate` and the `rp-2040` template.
