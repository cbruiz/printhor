extern crate proc_macro;
use proc_macro::TokenStream;
use quote::quote;
use syn::parse::{Parse, ParseStream};
use syn::{parse_macro_input, Expr, LitStr, Token, TypePath};

//#region The make_static_ref macro [...]

struct StaticInstanceParserInput {
    instance_name: LitStr,
    _comma1: Token![,],
    var_type: TypePath,
    _comma2: Token![,],
    owned_instance: Expr,
}

impl Parse for StaticInstanceParserInput {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let instance_name = input.parse()?;
        let _comma1 = input.parse()?;
        let var_type = input.parse()?;
        let _comma2 = input.parse()?;
        let owned_instance = input.parse()?;
        let _ = input.parse::<Token![,]>();
        Ok(StaticInstanceParserInput {
            instance_name,
            _comma1,
            var_type,
            _comma2,
            owned_instance,
        })
    }
}

/// Declares a static variable in .bss and returns a mutable reference to it.
///
/// This macro uses a wrapped implementation over the `static-cell` crate.
///
/// Syntax: [make_static_ref!] ( `instance_name`, `var_type`, `owned_instance` )
///
/// # Params
///
/// * `instance_name` - A `&str` literal for tracking purposes
/// * `var_type` - The type of the variable. Type must have the Size trait.
/// * `owned_instance` - The instance of the previous `var_type` to initialize the static.
///
/// # Usage
/// ```rust
/// use printhor_hwa_common_macros::make_static_ref;
/// // Or the convenient export of `printhor_hwa_common`
///
/// struct MyObject;
/// impl MyObject {
///     pub const fn new() -> Self { Self {} }
/// }
///
/// let instance: &'static mut MyObject = make_static_ref!(
///     "DummyStaticObject",
///     MyObject,
///     MyObject::new()
/// );
/// ```
#[proc_macro]
pub fn make_static_ref(input: TokenStream) -> TokenStream {
    let StaticInstanceParserInput {
        instance_name,
        var_type,
        owned_instance,
        ..
    } = parse_macro_input!(input as StaticInstanceParserInput);

    TokenStream::from(quote! {
        {
            #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
            #[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
            #[link_name = #instance_name]
            static CELL_INSTANCE: printhor_hwa_utils::StaticCell<#var_type> = printhor_hwa_utils::StaticCell::new();
            match printhor_hwa_utils::stack_allocation_increment(core::mem::size_of::<printhor_hwa_utils::StaticCell<#var_type>>()) {
                Ok(_num_bytes) => {
                    printhor_hwa_utils::debug!("Statically allocated {} bytes for {}", _num_bytes, #instance_name);
                    CELL_INSTANCE.init(#owned_instance)
                }
                Err(_num_bytes) => {
                    printhor_hwa_utils::error!("Unable to allocated {} bytes for {} (max: {}, actual: {})",
                        _num_bytes, #instance_name,
                        printhor_hwa_utils::MAX_STATIC_ALLOC_BYTES,
                        printhor_hwa_utils::stack_allocation_get(),
                    );
                    panic!("Unable to allocated {} bytes for {} (max: {}, actual: {})",
                        _num_bytes, #instance_name,
                        printhor_hwa_utils::MAX_STATIC_ALLOC_BYTES,
                        printhor_hwa_utils::stack_allocation_get(),
                    )
                }
            }

        }
    })
}

//#endregion

//#region The make_static_sync_controller macro [...]

struct StaticSyncControllerParserInput {
    instance_name: LitStr,
    _comma1: Token![,],
    mutex_strategy_type: TypePath,
    _comma2: Token![,],
    owned_instance: Expr,
}

impl Parse for StaticSyncControllerParserInput {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let instance_name = input.parse()?;
        let _comma1 = input.parse()?;
        let mutex_strategy_type = input.parse()?;
        let _comma2 = input.parse()?;
        let owned_instance = input.parse()?;
        let _ = input.parse::<Token![,]>();
        Ok(crate::StaticSyncControllerParserInput {
            instance_name,
            _comma1,
            mutex_strategy_type,
            _comma2,
            owned_instance,
        })
    }
}

/// Declares a instance of [StaticSyncController], which holds
/// a static reference of given resource wrapped into a `MutexStrategy` implementation.
/// The resource is allocated in .bss section.
///
/// This macro uses a wrapped implementation over the `static-cell` crate.
///
/// Syntax: [crate::make_static_sync_controller!] ( `instance_name`, `strategy_type`, `owned_instance` )
///
/// # Params
///
/// * `instance_name` - A `&str` literal for tracking purposes
/// * `strategy_type` - A concrete type implementing [SyncMutexStrategy] trait.
/// * `owned_instance` - The instance of the resource content.
///
/// # Usage
/// ```rust
/// use printhor_hwa_common_macros::make_static_sync_controller;
///
/// // Or the convenient export of `printhor_hwa_common`
///
/// struct MyObject;
/// impl MyObject {
///     pub const fn new() -> Self { Self {} }
/// }
///
/// let controller = make_static_sync_controller!(
///     "DummyController",
///     hwa::Holdable<hwa::SyncSendMutex, MyObject>,
///     MyObject::new()
/// );
/// ```
#[proc_macro]
pub fn make_static_sync_controller(input: TokenStream) -> TokenStream {
    let crate::StaticSyncControllerParserInput {
        instance_name,
        mutex_strategy_type,
        owned_instance,
        ..
    } = parse_macro_input!(input as StaticSyncControllerParserInput);

    let expanded = quote! {
        {
            type M = <#mutex_strategy_type as printhor_hwa_utils::SyncMutexStrategy>::SyncMutexType;
            type D = <#mutex_strategy_type as printhor_hwa_utils::SyncMutexStrategy>::Resource;

            #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
            #[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
            #[link_name = #instance_name]
            static CELL_INSTANCE: printhor_hwa_utils::StaticCell<printhor_hwa_utils::SyncMutex<M, D>> = printhor_hwa_utils::StaticCell::new();

            match printhor_hwa_utils::stack_allocation_increment(core::mem::size_of::<printhor_hwa_utils::StaticCell<printhor_hwa_utils::SyncMutex<M, D>>>()) {
                Ok(_num_bytes) => {
                    printhor_hwa_utils::debug!("Statically allocated {} bytes for {}", _num_bytes, #instance_name);
                    let controller: printhor_hwa_utils::StaticSyncController<#mutex_strategy_type> = printhor_hwa_utils::StaticSyncController::new(
                        <#mutex_strategy_type> :: new(
                            CELL_INSTANCE.init(
                                printhor_hwa_utils::SyncMutex::new(core::cell::RefCell::new(#owned_instance))
                            )
                        )
                    );
                    controller
                }
                Err(_num_bytes) => {
                    printhor_hwa_utils::error!("Unable to allocated {} bytes for {} (max: {}, actual: {})",
                        _num_bytes, #instance_name,
                        printhor_hwa_utils::MAX_STATIC_ALLOC_BYTES,
                        printhor_hwa_utils::stack_allocation_get(),
                    );
                    panic!("Unable to allocated {} bytes for {} (max: {}, actual: {})",
                        _num_bytes, #instance_name,
                        printhor_hwa_utils::MAX_STATIC_ALLOC_BYTES,
                        printhor_hwa_utils::stack_allocation_get(),
                    )
                }
            }
        }
    };
    TokenStream::from(expanded)
}

//#endregion

//#region "make_static_async_controller macro"

struct StaticAsyncControllerParserInput {
    instance_name: LitStr,
    _comma1: Token![,],
    mutex_strategy_type: TypePath,
    _comma2: Token![,],
    owned_instance: Expr,
}

impl Parse for StaticAsyncControllerParserInput {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let instance_name = input.parse()?;
        let _comma1 = input.parse()?;
        let mutex_strategy_type = input.parse()?;
        let _comma2 = input.parse()?;
        let owned_instance = input.parse()?;
        let _ = input.parse::<Token![,]>();
        Ok(StaticAsyncControllerParserInput {
            instance_name,
            _comma1,
            mutex_strategy_type,
            _comma2,
            owned_instance,
        })
    }
}

/// Declares a instance of [StaticAsyncController], which holds
/// a static reference of given resource wrapped into a [AsyncMutexStrategy] implementation.
/// The resource is allocated in .bss section.
///
/// This macro uses a wrapped implementation over the `static-cell` crate.
///
/// Syntax: [make_static_async_controller!] ( `instance_name`, `strategy_type`, `owned_instance` )
///
/// # Params
///
/// * `instance_name` - A `&str` literal for tracking purposes
/// * `strategy_type` - A concrete type implementing [AsyncMutexStrategy] trait.
/// * `owned_instance` - The instance of the resource content.
///
/// # Usage
/// ```rust
/// use printhor_hwa_common_macros::make_static_async_controller;
///
/// // Or the convenient export of `printhor_hwa_common`
///
/// struct MyObject;
/// impl MyObject {
///     pub const fn new() -> Self { Self {} }
/// }
///
/// let controller = make_static_async_controller!(
///     "DummyController",
///     hwa::Holdable<hwa::SyncSendMutex, MyObject>,
///     MyObject::new()
/// );
/// ```
#[proc_macro]
pub fn make_static_async_controller(input: TokenStream) -> TokenStream {
    let StaticAsyncControllerParserInput {
        instance_name,
        mutex_strategy_type,
        owned_instance,
        ..
    } = parse_macro_input!(input as StaticAsyncControllerParserInput);

    let expanded = quote! {
        {
            type M = <#mutex_strategy_type as printhor_hwa_utils::AsyncMutexStrategy>::AsyncMutexType;
            type D = <#mutex_strategy_type as printhor_hwa_utils::AsyncMutexStrategy>::Resource;

            #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
            #[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
            #[link_name = #instance_name]
            static CELL_INSTANCE: printhor_hwa_utils::StaticCell<printhor_hwa_utils::AsyncMutex<M, D>> = printhor_hwa_utils::StaticCell::new();

            match printhor_hwa_utils::stack_allocation_increment(core::mem::size_of::<printhor_hwa_utils::StaticCell<printhor_hwa_utils::AsyncMutex<M, D>>>()) {
                Ok(_num_bytes) => {
                    printhor_hwa_utils::debug!("Statically allocated {} bytes for {}", _num_bytes, #instance_name);
                    let controller: printhor_hwa_utils::StaticAsyncController<#mutex_strategy_type> = printhor_hwa_utils::StaticAsyncController::new(
                        <#mutex_strategy_type> :: new(
                            CELL_INSTANCE.init(
                                printhor_hwa_utils::AsyncMutex::new(#owned_instance)
                            )
                        )
                    );
                    controller
                }
                Err(_num_bytes) => {
                    printhor_hwa_utils::error!("Unable to allocated {} bytes for {} (max: {}, actual: {})",
                        _num_bytes, #instance_name,
                        printhor_hwa_utils::MAX_STATIC_ALLOC_BYTES,
                        printhor_hwa_utils::stack_allocation_get(),
                    );
                    panic!("Unable to allocated {} bytes for {} (max: {}, actual: {})",
                        _num_bytes, #instance_name,
                        printhor_hwa_utils::MAX_STATIC_ALLOC_BYTES,
                        printhor_hwa_utils::stack_allocation_get(),
                    )
                }
            }
        }
    };
    TokenStream::from(expanded)
}

//#endregion

//#region "log sink macro"

/// A sink utility to discard logs if no log feature is enabled.
#[proc_macro]
pub fn no_log(_input: TokenStream) -> TokenStream {
    TokenStream::from(quote! {{}})
}

//#endregion
