    TBB_INCLUDE_DIR = "../../external/tbb_x64-windows/include/"
    TBB_LIB_DIR = "../../external/tbb_x64-windows/lib/"
    TRACE_DIR = "../../external/minitrace/"

    project "TL"
        filename('%{prj.name}_'.._ACTION)
        kind "StaticLib"
        language "C++"
        warnings "Extra"
        objdir ('obj/'.._ACTION..'/%{cfg.architecture}')
        targetdir ( 'lib/' )
        files { "inc/*.h", "src/*.cpp" }
        includedirs { 'inc/', TRACE_DIR, TBB_INCLUDE_DIR }
        libdirs { TBB_LIB_DIR }
        vpaths { [("inc")] = {"inc/*.h"} }
        vpaths { [("src")] = {"src/*.cpp"} }
