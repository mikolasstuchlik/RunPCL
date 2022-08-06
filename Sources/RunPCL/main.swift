import CppPCL
import Foundation

// "/Users/mikolasstuchlik/Developer/RunPCL/Sources/RunPCL/Resources/Scan3.ply"
let path = Bundle.module.url(forResource: "Scan3", withExtension: "ply")!
publicFittingSurface(path.path)
