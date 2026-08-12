# Blaze patches

## `blaze-scoped-allocator.patch`

Applied at fetch time when `VORTEX_BLAZE_SCOPED_ALLOCATOR=ON`:

```sh
cmake -S . -B build -DVORTEX_BLAZE_SCOPED_ALLOCATOR=ON
```

It makes `blaze::AlignedAllocator` draw from vortex's active `helpers::memory_scope`, which is the
only way to reach blaze's *expression temporaries*. Blaze cannot be retargeted from the project
side: `DynamicMatrix`/`DynamicVector` hardwire `AllocatorType` to `AlignedAllocator` rather than
reporting their own `Alloc` parameter, and `GetAllocator` -- which every arithmetic trait consults
-- is an alias template, so it cannot be specialised either. With the patch on, even a plain
`blaze::DynamicMatrix<double>` allocates from the scope.

The patch touches one file. `alignedAllocate`/`alignedDeallocate` are called only from
`AlignedAllocator`, so `Memory.h` needs nothing.

### Both directions, or nothing

`allocate` and `deallocate` are patched together. Redirecting only the allocation side leaves
`alignedDeallocate` calling `free()` on memory a different allocator produced -- the
allocator/deallocator mismatch that shows up as a silent corruption on a lenient allocator and a
hard crash under a strict one such as Android's Scudo.

### It makes every blaze container scope-bound

Including ones that outlive the scope. `math::solve_ldlt` caches its LAPACK workspace in
`thread_local` storage; with the patch on and that workspace left on blaze's default allocator, it
holds a block of whatever arena was active during an earlier solve and releases it through that
arena at thread exit. That is a use-after-free, confirmed under ASan.

`math::heap_allocator` exists for exactly this: storage that must stay independent of any scope
names it explicitly. Anything long-lived added later needs the same treatment.
