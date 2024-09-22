#!/usr/bin/env bash
set -e

function run_test() {
    echo $1
    cargo clippy --examples --features $2 -- -Dwarnings
    cargo build --color=always --release --no-default-features \
            --features $2 \
            --target thumbv7em-none-eabihf
    cargo test --color=always --features $2
    cargo clean
}

cargo clippy
cargo test --color=always

run_test "Parse comment" parse-comments
run_test "Parse trailing comment" parse-trailing-comment
run_test "Combine comments & trailing comment" parse-trailing-comment,parse-comments

run_test "Parse checksum" parse-checksum
run_test "Parse checksum & trailing comment" parse-trailing-comment,parse-checksum
run_test "Parse checksum & comments & trailing comment" parse-trailing-comment,parse-comments,parse-checksum

run_test "Parse optional-value" optional-value
run_test "Parse string-value" string-value

run_test "Parse parameters" parse-parameters
run_test "Parse parameters & optional-value" parse-parameters,optional-value
run_test "Parse parameters & string-value" parse-parameters,string-value

run_test "Parse expressions" parse-expressions
run_test "Parse expressions & parameters" parse-expressions,parse-parameters

run_test "All features" parse-comments,parse-trailing-comment,parse-checksum,parse-parameters,parse-expressions,optional-value,string-value
