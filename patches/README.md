# Blaze patches

## `blaze-scoped-allocator.patch`

Applied unconditionally at configure time. It is vortex's memory model, not an option: with it
absent nothing is scope-allocated and `helpers::memory_scope` has no effect on blaze containers.

Applied after `FetchContent_MakeAvailable` rather than through `PATCH_COMMAND`, because that step
is skipped when blaze is supplied via `FETCHCONTENT_SOURCE_DIR_BLAZE` -- which would leave a build
silently unscoped. The marker is tested first, so reconfiguring is a no-op, and a failure to apply
is a hard configure error rather than a quiet fallback to stock blaze.

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
`thread_local` storage; sized under the caller's scope with the patch on, it holds a block of an
arena from an earlier solve and releases it at thread exit -- a use-after-free, confirmed under
ASan. The solvers size theirs back on `std::pmr::new_delete_resource` instead, and anything
long-lived added later needs a scope that outlives it too.
