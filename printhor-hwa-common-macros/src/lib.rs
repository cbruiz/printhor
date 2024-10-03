extern crate proc_macro;
use proc_macro::TokenStream;
use quote::quote;
use syn::parse::{Parse, ParseStream};
use syn::{parse_macro_input, Expr, LitStr, Token, TypePath};

//#region "make_static_ref macro"

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
/// use printhor_hwa_common as hwa;
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

//#region "make_static_controller macro"

struct StaticControllerParserInput {
    instance_name: LitStr,
    _comma1: Token![,],
    holder_type: TypePath,
    _comma2: Token![,],
    owned_instance: Expr,
}

impl Parse for StaticControllerParserInput {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let instance_name = input.parse()?;
        let _comma1 = input.parse()?;
        let holder_type = input.parse()?;
        let _comma2 = input.parse()?;
        let owned_instance = input.parse()?;
        let _ = input.parse::<Token![,]>();
        Ok(StaticControllerParserInput {
            instance_name,
            _comma1,
            holder_type,
            _comma2,
            owned_instance,
        })
    }
}

/// Declares a instance of [StaticController], which holds
/// a static reference of given resource wrapped into a `MaybeHoldable` implementation.
/// The resource is allocated in .bss section.
///
/// This macro uses a wrapped implementation over the `static-cell` crate.
///
/// Syntax: [make_static_controller!] ( `instance_name`, `holder_type`, `owned_instance` )
///
/// # Params
///
/// * `instance_name` - A `&str` literal for tracking purposes
/// * `holder_type` - A concrete type implementing [MayBeHoldable] trait.
/// * `owned_instance` - The instance of the resource content.
///
/// # Usage
/// ```rust
/// use printhor_hwa_common as hwa;
/// use printhor_hwa_common_macros::make_static_controller;
///
/// // Or the convenient export of `printhor_hwa_common`
///
/// struct MyObject;
/// impl MyObject {
///     pub const fn new() -> Self { Self {} }
/// }
///
/// let controller = make_static_controller!(
///     "DummyController",
///     hwa::Holdable<hwa::SyncSendMutex, MyObject>,
///     MyObject::new()
/// );
/// ```
#[proc_macro]
pub fn make_static_controller(input: TokenStream) -> TokenStream {
    let StaticControllerParserInput {
        instance_name,
        holder_type,
        owned_instance,
        ..
    } = parse_macro_input!(input as StaticControllerParserInput);


    let expanded = quote! {
        {
            type M = <#holder_type as printhor_hwa_utils::MaybeHoldable>::MutexType;
            type D = <#holder_type as printhor_hwa_utils::MaybeHoldable>::Resource;

            #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
            #[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
            #[link_name = #instance_name]
            static CELL_INSTANCE: printhor_hwa_utils::StaticCell<printhor_hwa_utils::Mutex<M, D>> = printhor_hwa_utils::StaticCell::new();

            match printhor_hwa_utils::stack_allocation_increment(core::mem::size_of::<printhor_hwa_utils::StaticCell<printhor_hwa_utils::Mutex<M, D>>>()) {
                Ok(_num_bytes) => {
                    printhor_hwa_utils::debug!("Statically allocated {} bytes for {}", _num_bytes, #instance_name);
                    let controller: printhor_hwa_utils::StaticController<#holder_type> = printhor_hwa_utils::StaticController::new(
                        <#holder_type> :: new(
                            CELL_INSTANCE.init(
                                printhor_hwa_utils::Mutex::new(#owned_instance)
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
