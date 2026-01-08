# Netpipe Migration Tests

This directory contains validation tests for netpipe streams before we fully migrate from ZMQ to netpipe.

## Test Files

### 1. test_netpipe_basic.cpp (228 lines)
**Purpose**: Validate basic netpipe stream functionality

**Tests**:
- TCP stream: connect, send, receive, echo
- IPC stream: Unix domain socket communication
- SHM stream: Shared memory ring buffer

**Data size**: Small messages (3-5 bytes)

**Status**: ✓ All tests pass

---

### 2. test_netpipe_concurrent.cpp (211 lines)
**Purpose**: Validate multiple simultaneous connections

**Tests**:
- TCP: 5 concurrent client connections
- IPC: 3 concurrent client connections with staggered timing
- Multi-threaded server handling each client in separate thread

**Data size**: Small messages (2-3 bytes per client)

**Status**: ✓ All tests pass (5/5 TCP clients, 3/3 IPC clients)

---

### 3. test_netpipe_large.cpp (326 lines)
**Purpose**: Validate large data transfers with throughput measurement

**Tests**:
- TCP: 1MB transfer with data integrity check
- IPC: 5MB transfer
- SHM: 10MB transfer with 16MB ring buffer

**Features**:
- Pattern-based data validation
- Throughput measurement (MB/s)
- Round-trip echo testing

**Status**: ✓ All tests pass
- TCP: ~1000 MB/s
- IPC: ~500-2500 MB/s
- SHM: ~450 MB/s

---

### 4. test_netpipe_xlarge.cpp (362 lines)
**Purpose**: Validate extra large data transfers (stress test)

**Tests**:
- TCP: 50MB transfer with multi-point integrity validation
- IPC: 100MB transfer
- SHM: 50MB transfer with 64MB ring buffer

**Features**:
- Large buffer allocation
- Multi-point data integrity checks
- Progress indicators for long operations
- Throughput measurement

**Status**: ✓ All tests pass
- TCP: ~750-2600 MB/s
- IPC: ~850-3400 MB/s
- SHM: High throughput via shared memory

---

### 5. test_netpipe_errors.cpp (272 lines)
**Purpose**: Validate error handling and edge cases

**Tests**:
- TCP connection refused (no server)
- TCP invalid hostname
- TCP port already in use
- TCP send after connection close
- TCP receive timeout/blocking
- IPC socket not found
- IPC invalid path
- SHM invalid size
- SHM not found
- Message size mismatch handling

**Status**: ✓ All error conditions properly detected and reported

---

## Building

The migration tests are automatically built when examples are enabled:

```bash
# Using xmake
xmake f --examples=y -y
xmake

# Using make
make build
```

Binaries are located in: `build/linux/x86_64/release/test_netpipe_*`

## Running

Run individual tests:

```bash
./build/linux/x86_64/release/test_netpipe_basic
./build/linux/x86_64/release/test_netpipe_concurrent
./build/linux/x86_64/release/test_netpipe_large
./build/linux/x86_64/release/test_netpipe_xlarge
./build/linux/x86_64/release/test_netpipe_errors
```

Or run all tests:

```bash
for test in build/linux/x86_64/release/test_netpipe_*; do
    echo "Running $test..."
    $test
    echo ""
done
```

## Results Summary

All netpipe streams (TCP, IPC, SHM) are validated and working:

✓ Basic functionality works correctly
✓ Concurrent connections handled properly
✓ Large data transfers (1-10MB) successful
✓ Extra large transfers (50-100MB) successful
✓ Error conditions properly detected and reported
✓ High throughput achieved (500-3400 MB/s depending on transport)

**Conclusion**: Netpipe is ready for integration into flatsim transport layer.

## Next Steps

1. Create `include/flatsim/transport.hpp` wrapper interface
2. Implement concrete transport classes (TcpTransport, IpcTransport, ShmTransport)
3. Update Simulator to use netpipe transport
4. Update Agent to use netpipe transport
5. Remove ZMQ dependency
