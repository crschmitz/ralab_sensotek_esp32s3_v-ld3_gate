import os

Import("env")

env.Append(
    LINKFLAGS=[
        "-flto",
        "-fuse-linker-plugin"
    ]
)

# Ensure that the compiler also uses LTO
env.Append(
    CXXFLAGS=[
        "-flto"
    ]
)
