# WebAssembly exception boundary

The WebAssembly module uses a module-local exception-safe Embind adapter in
`src/wasm/embind_boundary.js`. It targets **Emscripten 6.0.3**, the SDK pinned in
hosted CI. CMake rejects other SDK versions. The library rejects Wasm EH,
Asyncify/JSPI, memory64 and disabled exception catching at build time. Reviewing
this adapter and rerunning its tests is required before an SDK upgrade; its
hooks are SDK internals, not a general third-party Embind extension.

Emscripten's JS exception handling can leave the WASM stack lowered when a C++
exception escapes to JavaScript ([upstream issue 21606](https://github.com/emscripten-core/emscripten/issues/21606)).
The stock invoker also skips converted-argument destruction on that path.
Increasing stack capacity postpones this failure and does not release the
exception or argument allocations.

The adapter replaces only this module's Embind invoker generation, value-object
conversion and instance/static property wrappers. Constructors, methods, static
functions, free functions and overload entries use the invoker generator.
Each synchronous invocation saves/restores its stack and owns a separate
argument destructor list, so nested calls cannot overwrite cleanup state.
Value-object conversion cleans up partially constructed inputs and copied
outputs even if a field accessor fails. Returned class/shared handles retain
normal Embind ownership; callers still delete owned handles as documented.

Native `CppException` objects are caught once, marked caught using the SDK's
`__cxa_begin_catch`, copied into an ordinary JavaScript `Error`, then released by
`__cxa_end_catch`. This balances both the uncaught-exception counter and native
reference count. The error's name contains the native exception type and its
message contains the copied native message. It has no native pointer to free.
Ordinary JavaScript exceptions retain identity, including thrown primitive
values. No global functions or unrelated modules are patched.

A foreign JavaScript exception raised through `emscripten::val` inside native
code does **not** generally unwind native RAII objects under JS EH. Consequently,
the array setters `WaveMembrane.setState`, `PeriodicMacGrid.setVelocities` and
`MaxwellGrid.setState`, `ElasticWaveGrid.setState`, and the periodic scalar
state/velocity setters first snapshot their inputs in JavaScript. The static
electrostatic solve `PeriodicElectrostaticGrid.solve` captures its charge array and all four
primitive option fields before native receiver wiring, so an options getter may
also safely throw, reenter or delete the receiver. Captured native
sizing observers, rather than user-shadowed methods, bound the copy to the
receiver's allocated grid size (with the 262144 MAC/Maxwell/elastic/scalar/electrostatic hard cell cap).
All array shapes are checked before reading any entries. Entries must be own,
dense, finite numbers; typed arrays are not accepted. Accessors and proxy traps
may throw before native conversion/allocation, and reentrant receiver deletion
is detected during subsequent normal pointer validation. These operations add
a bounded JavaScript array copy to the existing native transactional copy.
This does not promise RAII safety for arbitrary future C++ calls into foreign
JavaScript, user-modified global intrinsics, asynchronous callbacks or traps.
New native-to-JS access paths need the same explicit boundary review.

Configure `-DPHYSICS_WASM_BOUNDARY_TEST_PROBES=ON` to add test-only bindings;
production builds default to OFF. Run both:

```sh
node build-wasm/wasm/smoke-test.cjs
node build-wasm/wasm/boundary-stress.cjs
```

The stress test warms allocator paths, then checks exact live allocation bytes,
stack pointer, native uncaught count, converted-value lifetimes and owned-object
lifetimes across 2000 mixed exception/conversion/constructor/property/overload
batches, 1000 array-accessor/proxy/reentrant batches and 1000 receiver-deletion
batches. It also verifies JavaScript error identity and copied return values.
The production physics smoke suite runs in its existing order; no larger stack
or test-only stack reset is used. Hosted WASM CI enables and runs these probes.

The adapter's copied/adapted SDK portions retain Emscripten's copyright and MIT
license in `src/wasm/embind_boundary.LICENSE`.
