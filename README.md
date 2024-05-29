# VEGA PROJECT - AIR QUALITY

## Run example for ZMOD4510, BME688, AS7331 sensors and BLE advertising:

```
cargo run --bin vega-zmod
```
```
cargo run --bin vega-bme
```
```
cargo run --bin vega-as7331
```
```
cargo run --bin ble
```

## Vega project - read data from ZMOD4510, BME688, AS7331 and BLE advertising:
```
cargo run --bin vega --features "all"
```
- features: ```zmod4510, bme688, as7331, all```

## Advertising format:
```
<temperature(1byte)> <humidity(1byte)> <pressure(1byte)> <gas resistance (4 bytes)> <UV_A (4 bytes)> <UV_B (4 bytes)> <UV_C (4 bytes)> <O3 (2 bytes)> <Fast AQI (2 bytes)>
```
**Total: 23 bytes**
- temperature 0-100 (celsius)
- humidity 0-100 (%)
- gas resistance: float (f32) (kOhm)
- UV_A: float (f32) (uW/cm^2)
- UV_B: float (f32) (uW/cm^2)
- UV_C: float (f32) (uW/cm^2)
- O3: 0-2000 (ppb)
- fast AQI: 0-500
## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
