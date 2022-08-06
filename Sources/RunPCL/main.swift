import CppPCL
import Foundation

//print("Hello, world!")
//greetMe()
//tryBoost()
//runPcl("/Users/mikolasstuchlik/Downloads/RunPCL/RunPCL/Scan3.ply")
let path = Bundle.module.url(forResource: "Scan3", withExtension: "ply")!
publicFittingSurface(path.path)
