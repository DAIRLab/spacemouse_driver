def _x11_repo_impl(repo_ctx):
    repo_ctx.symlink(repo_ctx.path("/opt/X11/include"), "include")
    repo_ctx.symlink(repo_ctx.path("/opt/X11/lib"), "lib")

    repo_ctx.file("BUILD.bazel", """
cc_library(
    name = "X11",
    hdrs = glob(["include/X11/**/*.h"]),
    includes = ["include"],
    linkopts = ["-L/opt/X11/lib"],
    visibility = ["//visibility:public"],
)
""")
    repo_ctx.file("MODULE.bazel", 'module(name="X11", version="0.0.1")')

x11_repo = repository_rule(
    implementation = _x11_repo_impl,
    local = True,
)

def _x11_stub_repo_impl(repo_ctx):
    repo_ctx.file("BUILD.bazel", """
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "X11",
)
""")

x11_stub_repo = repository_rule(
    implementation = _x11_stub_repo_impl,
)

def _x11_extension_impl(module_ctx):
    os = module_ctx.os.name
    if os == "mac os x":
        x11_repo(name = "X11")
    else:
        x11_stub_repo(name = "X11")

x11_extension = module_extension(
    implementation = _x11_extension_impl,
)
