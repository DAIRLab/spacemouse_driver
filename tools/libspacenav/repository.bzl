load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _libspnav_extension_impl(module_ctx):
    http_archive(
        name = "libspnav",
        build_file = "@spacemouse//tools/libspacenav:package.BUILD.bazel",
        sha256 = "093747e7e03b232e08ff77f1ad7f48552c06ac5236316a5012db4269951c39db",
        strip_prefix = "libspnav-1.2",
        urls = ["https://github.com/FreeSpacenav/libspnav/releases/download/v1.2/libspnav-1.2.tar.gz"],
    )

# The extension definition is now much simpler without any tag_classes
libspnav_extension = module_extension(
    implementation = _libspnav_extension_impl,
)
