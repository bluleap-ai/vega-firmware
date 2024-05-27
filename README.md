# VEGA PROJECT - AIR QUALITY

## Run example for ZMOD4510, BME688, AS7331 sensors:

```
cargo run --bin vega-zmod
```
```
cargo run --bin vega-bme
```
```
cargo run --bin vega-as7331
```
## Advertising format:
```
<temperature(1byte)> <humidity(1byte)> <pressure(1byte)> <gas resistance (4 bytes)> <UV_A (4 bytes)> <UV_B (4 bytes)> <UV_C (4 bytes)> <O3 (2 bytes)> <Fast AQI (2 bytes)>
```
**Total: 23 bytes**
## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
