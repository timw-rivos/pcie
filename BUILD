# SPDX-FileCopyrightText: Copyright (c) 2023 by Rivos Inc.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

# PCI library

package(default_visibility = ["//visibility:public"])

load("@bazel_skylib//rules:common_settings.bzl", "string_flag")
load("@rules_rust//rust:defs.bzl", "rust_library")

#
# Compile-time features
#

rust_library(
    name = "pci",
    srcs = glob(
        ["src/*.rs"],
    ),
    edition = "2021",
    deps = [],
)
