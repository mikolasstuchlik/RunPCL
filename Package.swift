// swift-tools-version: 5.6
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let pclRoot = "/Users/mikolasstuchlik/Developer/pcl"
let pclBuildRoot = "/Users/mikolasstuchlik/Developer/pcl/build"
let libInstallPath = "/usr/local/Cellar"

// https://dev.my-gate.net/2021/08/04/understanding-rpath-with-cmake/
// swift run -Xlinker -rpath -Xlinker /Users/mikolasstuchlik/Developer/pcl/build/lib

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
                "-I\(libInstallPath)/boost/1.78.0_1/include",
                "-I\(libInstallPath)/eigen/3.4.0_1/include/eigen3",
                "-I\(libInstallPath)/flann/1.9.1_13/include",
                "-I\(pclRoot)/common/include",
				"-I\(pclRoot)/io/include",
				"-I\(pclRoot)/build/include",
				"-I\(pclRoot)/kdtree/include",
				"-I\(pclRoot)/features/include",
				"-I\(pclRoot)/surface/include",
				"-I\(pclRoot)/search/include",
            ])],
            // All symbols that were used have to be defined.
            // The linker needs to know, where to search for your definitions
            linkerSettings: [.unsafeFlags([
                // Manual:
                // -L <dir> Add directory to library search path
                // Comment:
                // You can either provide full paths to libraries, or provide paths, where libraries are stored.
                "-L\(libInstallPath)/boost/1.78.0_1/lib",
                "-L\(libInstallPath)/flann/1.9.1_13/lib",
                "-L\(pclBuildRoot)/lib",
                // Comment:
                // -l<library>
                // You can either provide full paths to libraries, or use this command to find library of the provided name
                // on default paths, or additional paths provided above using `-L` command.
                //
                // Suppose you want to link library named Miki. Then provide additional flag `-lMiki`. The linker will then search
                // for library named `libMiki.dylib` (dynamic library) or `libMiki.a` (static library).
                // Notice, that it is not explicitly defined whether static or dynamic library is used.
                "-lpcl_io",
                "-lboost_prg_exec_monitor-mt",
                "-lpcl_octree",
                "-lpcl_features",
                "-lpcl_surface",
                "-lpcl_search",
                "-lpcl_common",
            ])]
        ),
        .executableTarget(
            name: "RunPCL", 
            dependencies: ["CppPCL"]
        ),
    ],
    cxxLanguageStandard: .cxx17
)

// Make static
// cmake \
// -D BUILD_surface_on_nurbs:BOOL=ON \
// -D CMAKE_BUILD_TYPE:STRING=Release \
// -D CMAKE_CXX_FLAGS_RELEASE:STRING="-Ofast -DNDEBUG -funroll-loops -fno-strict-aliasing -ftree-vectorize -fomit-frame-pointer -march=native" \
// -D CMAKE_C_FLAGS_RELEASE:STRING="-Ofast -DNDEBUG  -funroll-loops -fno-strict-aliasing -ftree-vectorize -fomit-frame-pointer -march=native" \
// -D PCL_SHARED_LIBS=OFF \
// -D PCL_FLANN_REQUIRED_TYPE:STRING=STATIC \
// -D PCL_QHULL_REQUIRED_TYPE:STRING=STATIC \
// ..

// Make dynamic
// cmake \
// -D BUILD_surface_on_nurbs:BOOL=ON \
// -D CMAKE_BUILD_TYPE:STRING=Release \
// -D CMAKE_CXX_FLAGS_RELEASE:STRING="-Ofast -DNDEBUG -funroll-loops -fno-strict-aliasing -ftree-vectorize -fomit-frame-pointer -march=native" \
// -D CMAKE_C_FLAGS_RELEASE:STRING="-Ofast -DNDEBUG  -funroll-loops -fno-strict-aliasing -ftree-vectorize -fomit-frame-pointer -march=native" \
// ..