extern crate proc_macro;
use proc_macro::TokenStream;
use quote::quote;
use syn::parse::{Parse, ParseStream};
use syn::{Expr, LitStr, Token, TypePath, parse_macro_input};

//#region "The make_static_ref macro"

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
            #[cfg_attr(not(target_arch = "aarch64"), unsafe(link_section = ".bss"))]
            #[link_name = #instance_name]
            static CELL_INSTANCE: printhor_hwa_utils::StaticCell<#var_type> = printhor_hwa_utils::StaticCell::new();
            match printhor_hwa_utils::stack_allocation_increment(core::mem::size_of::<printhor_hwa_utils::StaticCell<#var_type>>()) {
                Ok(_num_bytes) => {
                    printhor_hwa_utils::debug!("Statically allocated {} bytes for {}", _num_bytes, #instance_name);
                    CELL_INSTANCE.init(#owned_instance)
                }
                Err(_num_bytes) => {
                    printhor_hwa_utils::error!("Unable to allocated {} bytes for {} (actual: {})",
                        _num_bytes, #instance_name,
                        printhor_hwa_utils::stack_allocation_get(),
                    );
                    panic!("Unable to allocated {} bytes for {} (actual: {})",
                        _num_bytes, #instance_name,
                        printhor_hwa_utils::stack_allocation_get(),
                    )
                }
            }

        }
    })
}

//#endregion

//#region "The make_static_sync_controller macro"

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

/// Declares an instance of printhor_hwa_common::StaticSyncController, which holds
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
/// * `strategy_type` - A concrete type implementing printhor_hwa_common::SyncMutexStrategy trait.
/// * `owned_instance` - The instance of the resource content.
///
/// # Usage
/// ```rust
/// use printhor_hwa_common as hwa;
/// use hwa::make_static_sync_controller;
///
/// // Or the convenient export of `printhor_hwa_common`
///
/// struct MyObject;
/// impl MyObject {
///     pub const fn new() -> Self { Self {} }
/// }
///
/// let controller = hwa::make_static_sync_controller!(
///     "DummyController",
///     hwa::SyncStandardStrategy<hwa::SyncCsMutexType, MyObject>,
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

            #[cfg_attr(not(target_arch = "aarch64"), unsafe(link_section = ".bss"))]
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
                    printhor_hwa_utils::error!("Unable to allocated {} bytes for {} (actual: {})",
                        _num_bytes, #instance_name,
                        printhor_hwa_utils::stack_allocation_get(),
                    );
                    panic!("Unable to allocated {} bytes for {} (actual: {})",
                        _num_bytes, #instance_name,
                        printhor_hwa_utils::stack_allocation_get(),
                    )
                }
            }
        }
    };
    TokenStream::from(expanded)
}

//#endregion

//#region "The make_static_async_controller macro"

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

/// Declares an instance of printhor_hwa_common::StaticAsyncController, which holds
/// a static reference of given resource wrapped into a printhor_hwa_common::AsyncMutexStrategy implementation.
/// The resource is allocated in .bss section.
///
/// This macro uses a wrapped implementation over the `static-cell` crate.
///
/// Syntax: [make_static_async_controller!] ( `instance_name`, `strategy_type`, `owned_instance` )
///
/// # Params
///
/// * `instance_name` - A `&str` literal for tracking purposes
/// * `strategy_type` - A concrete type implementing printhor_hwa_utils::AsyncMutexStrategy trait.
/// * `owned_instance` - The instance of the resource content.
///
/// # Usage
/// ```rust
/// use printhor_hwa_common as hwa;
/// use hwa::make_static_async_controller;
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
///     hwa::AsyncHoldableStrategy<hwa::AsyncNoopMutexType, MyObject>,
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

            #[cfg_attr(not(target_arch = "aarch64"), unsafe(link_section = ".bss"))]
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
                    printhor_hwa_utils::error!("Unable to allocated {} bytes for {} (actual: {})",
                        _num_bytes, #instance_name,
                        printhor_hwa_utils::stack_allocation_get(),
                    );
                    panic!("Unable to allocated {} bytes for {} (actual: {})",
                        _num_bytes, #instance_name,
                        printhor_hwa_utils::stack_allocation_get(),
                    )
                }
            }
        }
    };
    TokenStream::from(expanded)
}

//#endregion

//#region "The log sink macro"

/// A sink utility to discard logs if no log feature is enabled.
#[proc_macro]
pub fn no_log(_input: TokenStream) -> TokenStream {
    TokenStream::from(quote! {{}})
}

//#endregion

//#region "The TVector macro(s)"

struct KeyValue {
    key: syn::Ident,
    _eq_token: Token![=],
    value: Expr,
}

/// Implementa el parseo de clave-valor
impl Parse for KeyValue {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        Ok(KeyValue {
            key: input.parse()?,
            _eq_token: input.parse()?,
            value: input.parse()?,
        })
    }
}

struct KeyValueList(Vec<KeyValue>);

impl Parse for KeyValueList {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut key_values = Vec::new();
        while !input.is_empty() {
            key_values.push(input.parse()?);
            if input.peek(Token![,]) {
                let _ = input.parse::<Token![,]>();
            }
        }
        Ok(KeyValueList(key_values))
    }
}

/// Syntactic sugar for creating const vectors
#[proc_macro]
pub fn make_vector(input: TokenStream) -> TokenStream {
    let KeyValueList(pairs) = match syn::parse::<KeyValueList>(input) {
        Ok(parsed) => parsed,
        Err(err) => return err.to_compile_error().into(),
    };

    // HashMap para almacenar las expresiones
    let mut expr_map = std::collections::HashMap::new();

    // Lista de ejes válidos
    let valid_axes = [
        "e", "x", "y", "z", "a", "b", "c", "i", "j", "k", "u", "v", "w",
    ];

    for &axis in &valid_axes {
        expr_map.insert(axis.to_string(), quote! { None });
    }

    for pair in pairs {
        let axis = &pair.key;
        let axis_name = axis.to_string();
        let value = &pair.value;

        if !valid_axes.contains(&axis_name.as_str()) {
            return syn::Error::new(
                axis.span(),
                format!(
                    "Unexpected key `{}`, expected one of: {}",
                    axis_name,
                    valid_axes.join(", ")
                ),
            )
            .to_compile_error()
            .into();
        }
        expr_map.insert(axis_name.clone(), quote! { Some(#value) });
    }

    let e_expr = expr_map.remove("e").unwrap();
    //
    let x_expr = expr_map.remove("x").unwrap();
    let y_expr = expr_map.remove("y").unwrap();
    let z_expr = expr_map.remove("z").unwrap();
    //
    let a_expr = expr_map.remove("a").unwrap();
    let b_expr = expr_map.remove("b").unwrap();
    let c_expr = expr_map.remove("c").unwrap();
    //
    let i_expr = expr_map.remove("i").unwrap();
    let j_expr = expr_map.remove("j").unwrap();
    let k_expr = expr_map.remove("k").unwrap();
    //
    let u_expr = expr_map.remove("u").unwrap();
    let v_expr = expr_map.remove("v").unwrap();
    let w_expr = expr_map.remove("w").unwrap();

    // Generar el código de salida
    let output = quote! {
        printhor_hwa_common::math::TVector::from_coords(
            #[cfg(feature="with-e-axis")] #e_expr,
            //
            #[cfg(feature="with-x-axis")] #x_expr,
            #[cfg(feature="with-y-axis")] #y_expr,
            #[cfg(feature="with-z-axis")] #z_expr,
            //
            #[cfg(feature="with-a-axis")] #a_expr,
            #[cfg(feature="with-b-axis")] #b_expr,
            #[cfg(feature="with-c-axis")] #c_expr,
            //
            #[cfg(feature="with-i-axis")] #i_expr,
            #[cfg(feature="with-j-axis")] #j_expr,
            #[cfg(feature="with-k-axis")] #k_expr,
            //
            #[cfg(feature="with-u-axis")] #u_expr,
            #[cfg(feature="with-v-axis")] #v_expr,
            #[cfg(feature="with-w-axis")] #w_expr,
        )

    };

    output.into()
}

/// Syntactic sugar for creating const vectors
#[proc_macro]
pub fn make_vector_real(input: TokenStream) -> TokenStream {
    let KeyValueList(pairs) = match syn::parse::<KeyValueList>(input) {
        Ok(parsed) => parsed,
        Err(err) => return err.to_compile_error().into(),
    };

    // HashMap para almacenar las expresiones
    let mut expr_map = std::collections::HashMap::new();

    // Lista de ejes válidos
    let valid_axes = [
        "e", "x", "y", "z", "a", "b", "c", "i", "j", "k", "u", "v", "w",
    ];

    for &axis in &valid_axes {
        expr_map.insert(axis.to_string(), quote! { None });
    }

    for pair in pairs {
        let axis = &pair.key;
        let axis_name = axis.to_string();
        let value = &pair.value;

        if !valid_axes.contains(&axis_name.as_str()) {
            return syn::Error::new(
                axis.span(),
                format!(
                    "Unexpected key `{}`, expected one of: {}",
                    axis_name,
                    valid_axes.join(", ")
                ),
            )
            .to_compile_error()
            .into();
        }
        expr_map.insert(axis_name.clone(), quote! { Some(hwa::make_real!( #value)) });
    }

    let e_expr = expr_map.remove("e").unwrap();
    //
    let x_expr = expr_map.remove("x").unwrap();
    let y_expr = expr_map.remove("y").unwrap();
    let z_expr = expr_map.remove("z").unwrap();
    //
    let a_expr = expr_map.remove("a").unwrap();
    let b_expr = expr_map.remove("b").unwrap();
    let c_expr = expr_map.remove("c").unwrap();
    //
    let i_expr = expr_map.remove("i").unwrap();
    let j_expr = expr_map.remove("j").unwrap();
    let k_expr = expr_map.remove("k").unwrap();
    //
    let u_expr = expr_map.remove("u").unwrap();
    let v_expr = expr_map.remove("v").unwrap();
    let w_expr = expr_map.remove("w").unwrap();

    // Generar el código de salida
    let output = quote! {
        hwa::math::TVector::from_coords(
            #[cfg(feature="with-e-axis")] #e_expr,
            //
            #[cfg(feature="with-x-axis")] #x_expr,
            #[cfg(feature="with-y-axis")] #y_expr,
            #[cfg(feature="with-z-axis")] #z_expr,
            //
            #[cfg(feature="with-a-axis")] #a_expr,
            #[cfg(feature="with-b-axis")] #b_expr,
            #[cfg(feature="with-c-axis")] #c_expr,
            //
            #[cfg(feature="with-i-axis")] #i_expr,
            #[cfg(feature="with-j-axis")] #j_expr,
            #[cfg(feature="with-k-axis")] #k_expr,
            //
            #[cfg(feature="with-u-axis")] #u_expr,
            #[cfg(feature="with-v-axis")] #v_expr,
            #[cfg(feature="with-w-axis")] #w_expr,
        )

    };

    output.into()
}

fn do_make_real(input: TokenStream) -> TokenStream {
    let number: syn::LitFloat = syn::parse(input).unwrap();
    cfg_if::cfg_if! {
        if #[cfg(feature="fixed-point-128-impl")] {
            let expanded = quote! {
                hwa::math::Real::from_fixed(rust_decimal_macros::dec!(#number))
            };
            expanded.into()
        }
        else if #[cfg(feature="float-point-f64-impl")] {
            let expanded = quote! {
                hwa::math::Real::from_f64(#number)
            };
            expanded.into()
        }
        else {
            // assuming #[cfg(feature="float-point-f32-impl")]
            let expanded = quote! {
                hwa::math::Real::from_f32(#number)
            };
            expanded.into()
        }
    }
}

#[proc_macro]
/// Syntactic sugar for creating const optional real literals
pub fn make_optional_real(input: TokenStream) -> TokenStream {
    let expanded = if input.is_empty() {
        quote! {None}
    } else {
        let number: syn::LitFloat = syn::parse(input).unwrap();
        quote! {Some( hwa::make_real!(#number))}
    };

    expanded.into()
}

#[proc_macro]
/// Syntactic sugar for creating const real literals
pub fn make_real(input: TokenStream) -> TokenStream {
    do_make_real(input).into()
}

//#endregion
