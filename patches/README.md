# Blaze patches

## `blaze-scoped-allocator.patch`

Applied at fetch time when `VORTEX_BLAZE_SCOPED_ALLOCATOR=ON`:

```sh
cmake -S . -B build -DVORTEX_BLAZE_SCOPED_ALLOCATOR=ON
```

It makes `blaze::AlignedAllocator` draw from vortex's active `helpers::memory_scope`, the only way
to reach blaze's *expression temporaries*: the containers hardwire `AllocatorType` to
`AlignedAllocator` rather than reporting their own `Alloc`, and `GetAllocator` is an alias template
that cannot be specialised, so blaze cannot be retargeted from the project side. With the patch on,
even a plain `blaze::DynamicMatrix<double>` allocates from the scope.

One file: `alignedAllocate`/`alignedDeallocate` are called only from `AlignedAllocator`.

### Both directions, or nothing

`allocate` and `deallocate` are patched together. Redirecting only allocation leaves
`alignedDeallocate` calling `free()` on memory another allocator produced -- silent corruption on a
lenient allocator, a hard crash under a strict one such as Android's Scudo.

### It makes every blaze container scope-bound

Including ones that outlive the scope. `math::solve_ldlt` caches its LAPACK workspace in
`thread_local` storage; left on blaze's default allocator with the patch on, it holds a block of an
arena from an earlier solve and releases it at thread exit -- a use-after-free, confirmed under
ASan. `math::heap_allocator` exists for exactly that, and anything long-lived added later needs it
too.
