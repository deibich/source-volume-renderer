# raw to openvdb

Requires compiler flag `/std:c++17`.

```bash
vcpkg install zlib:x64-windows
vcpkg install blosc:x64-windows
vcpkg install tbb:x64-windows
vcpkg install boost-iostreams:x64-windows
vcpkg install boost-any:x64-windows
vcpkg install boost-algorithm:x64-windows
vcpkg install boost-uuid:x64-windows
vcpkg install boost-interprocess:x64-windows
vcpkg install openvdb:x64-windows
```

## Usage

```bash
.\raw2vdb.exe .\data\bonsai_256x256x256_uint8.raw uint8 256 256 256 .\out\bonsai.vdb
```