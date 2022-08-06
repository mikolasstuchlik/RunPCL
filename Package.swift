// swift-tools-version: 5.6
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

/// Location of the PCL repository
let pclRoot = "/Users/mikolasstuchlik/Developer/pcl"
/// Location of the build folder
let pclBuildRoot = "/Users/mikolasstuchlik/Developer/pcl/build_dy"

// Include paths for homebrew repositories differ based on the architecture
let m1libIncludePath = "/opt/homebrew/Cellar"
let x86libIncludePath = "/usr/local/Cellar"

let m1llvmIncludePath = "/opt/homebrew/opt/llvm/include"
let x86llvmIncludePath = "/opt/homebrew/opt/llvm/include" // check via `$ brew info llvm`

let m1libBinaryPath = "/opt/homebrew/Cellar"
let x86libBinaryPath = "/usr/local/Cellar"

let m1llvmBinaryPath = "/opt/homebrew/opt/llvm/lib"
let x86llvmBinaryPath = "/opt/homebrew/opt/llvm/lib" // check via `$ brew info llvm`

#if arch(arm64)
let includePath = m1libIncludePath
let binaryPath = m1libBinaryPath
let llvmIncludePath = m1llvmIncludePath
let llvmBinaryPath = m1llvmBinaryPath
#else
let includePath = x86libIncludePath
let binaryPath = x86libBinaryPath
let llvmIncludePath = x86llvmIncludePath
let llvmBinaryPath = x86llvmBinaryPath
#endif

// Statically linked:
// https://www.positioniseverything.net/clang-error-unsupported-option-fopenmp/ 
// You need to install LLVM as described in `– Linker Errors After Compilation`

// You can use `$ swift package generate-xcodeproj` in order to generate Xcode project
// The advantage of this is, that the SourceKit works, since it can find all the deader files.
// The disadvantage is, that Bundle.module won't work, you need to add resources in different way.

let package = Package(
    name: "RunPCL",
    targets: [
        .target(
            name: "CppPCL",
            cxxSettings: [.unsafeFlags([
                // Manual:
                // -I <dir> Add directory to the end of the list of include search paths
                // Comment:
                // Each symbol (function declaration) you want to use nees to be .. declared.
                // You can write those declarations yourself, or include additional header files in your project.
                // Header files will be searched additionaly in folowing paths:
                "-I\(includePath)/boost/1.79.0_1/include",
                "-I\(includePath)/eigen/3.4.0_1/include/eigen3",
                "-I\(includePath)/flann/1.9.1_13/include",
                "-I\(pclRoot)/common/include",
                "-I\(pclRoot)/io/include",
                "-I\(pclRoot)/build/include",
                "-I\(pclRoot)/kdtree/include",
                "-I\(pclRoot)/features/include",
                "-I\(pclRoot)/surface/include",
                "-I\(pclRoot)/search/include",
                "-I\(llvmIncludePath)"
            ])],
            // All symbols that were used have to be defined.
            // The linker needs to know, where to search for your definitions
            linkerSettings: [.unsafeFlags([
                // Manual:
                // -L <dir> Add directory to library search path
                // Comment:
                // You can either provide full paths to libraries, or provide paths, where libraries are stored.
                "-L\(binaryPath)/boost/1.79.0_1/lib",
                "-L\(binaryPath)/flann/1.9.1_13/lib",
                "-L\(pclBuildRoot)/lib",
                "-L\(llvmBinaryPath)",
                // Manual (man rd):
                //-rpath path
                // Add path to the runpath search path list for image being created.  At runtime, dyld uses the runpath when searching for
                // dylibs whose load path begins with @rpath/. 
                "-Xlinker", "-rpath", "-Xlinker", "\(pclBuildRoot)/lib",
                // Comment:
                // -l<library>
                // You can either provide full paths to libraries, or use this command to find library of the provided name
                // on default paths, or additional paths provided above using `-L` command.
                //
                // Suppose you want to link library named Miki. Then provide additional flag `-lMiki`. The linker will then search
                // for library named `libMiki.dylib` (dynamic library) or `libMiki.a` (static library).
                // Notice, that it is not explicitly defined whether static or dynamic library is used.
                "-lomp",
                "-lflann",
                "-lflann_cpp",
                "-lboost_prg_exec_monitor-mt",
                "-lboost_filesystem",
                "-lpcl_io_ply",
                "-lpcl_io",
                "-lpcl_octree",
                "-lpcl_kdtree",
                "-lpcl_features",
                "-lpcl_surface",
                "-lpcl_search",
                "-lpcl_common",
            ])]
        ),
        .executableTarget(
            name: "RunPCL", 
            dependencies: ["CppPCL"],
            resources: [
                .process("Resources")
            ]
        ),
    ],
    cxxLanguageStandard: .cxx17
)
