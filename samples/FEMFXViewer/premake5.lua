SOLUTION_NAME = "FEMFXViewer"
FEMFX_LIB_NAME = "AMD_FEMFX"
TASK_SYSTEM_NAME = "TL"

solution ( SOLUTION_NAME )

FEMFX_BASE_DIR = "../../"
FEMFX_SRC_DIR = FEMFX_BASE_DIR.."amd_femfx/src/"
FEMFX_INC_DIR = FEMFX_BASE_DIR.."amd_femfx/inc/"
FEMFX_LIB_DIR = FEMFX_BASE_DIR.."amd_femfx/lib/"
VECTORMATH_INC_DIR = FEMFX_BASE_DIR.."amd_femfx/inc/Vectormath/"
TRACE_DIR = FEMFX_BASE_DIR.."external/minitrace/"
SAMPLES_COMMON_DIR = FEMFX_BASE_DIR.."samples/common/"
JSON_INC_DIR = FEMFX_BASE_DIR.."external/json/include"
TASK_SYSTEM_DIR = FEMFX_BASE_DIR.."amd_femfx/TL/"
TASK_SYSTEM_INC_DIR = TASK_SYSTEM_DIR.."inc/"

-- Solution-wide config

    filename(SOLUTION_NAME.."_".._ACTION)
    configurations { "Release", "Debug" }
    platforms { "x64" }
    symbols "On"
    architecture "x86_64"
    floatingpoint "Fast"
    vectorextensions "AVX2"
    exceptionhandling ("Off")
    --callingconvention ("VectorCall")

    defines { 
        "WIN32", 
        "NOMINMAX",
        "MTR_ENABLED"
	}

    objdir ('obj/'.._ACTION..'/%{cfg.architecture}')
    targetdir ( 'build/'.._ACTION..'/%{cfg.architecture}/%{cfg.buildcfg}' )

    filter "configurations:Debug" 
        defines { "_DEBUG" }
        targetsuffix("_d")

    filter "configurations:Release"
        defines { "NDEBUG" }
        optimize "Speed"

	filter "action:vs2017"
		systemversion "10.0.17763.0"
    
    EXE_NAME = "FEMFXViewer"
    SRC_DIR = ""
    EXAMPLE_RB_DIR = FEMFX_BASE_DIR.."samples/ExampleRigidBodies/"
    GLFW_VS_INCLUDE_DIR = FEMFX_BASE_DIR.."external/glfw/include/"
    GLFW_VS_LIB_DIR = FEMFX_BASE_DIR.."external/glfw/lib-vc2022/"
    GLFW_GMAKE_BASE_DIR = "/ucrt64/"
    --GLFW_GMAKE_BASE_DIR = "/usr/local/"

    project (EXE_NAME)
        filename('%{prj.name}_'.._ACTION)
        targetname(EXE_NAME)
        kind "ConsoleApp"
        language "C++"
        warnings "Extra"
		buildoptions { '$(FEMFXViewerOptionsEnvVar)' }
        files { SRC_DIR.."*.cpp", SRC_DIR.."*.h", SAMPLES_COMMON_DIR.."*.cpp", SAMPLES_COMMON_DIR.."*.h" }
        files { EXAMPLE_RB_DIR.."*.cpp", EXAMPLE_RB_DIR.."*.h" }     
        --files { TRACE_DIR.."minitrace.c", TRACE_DIR.."minitrace.h" }
        vpaths { [("ExampleRigidBodies")] = {EXAMPLE_RB_DIR.."**.h", EXAMPLE_RB_DIR.."**.cpp"} }
        vpaths { [("Trace")] = {TRACE_DIR.."**.h", TRACE_DIR.."**.c"} }
        vpaths { [("")] = { SRC_DIR.."*.cpp", SRC_DIR.."*.h", SAMPLES_COMMON_DIR.."*.cpp", SAMPLES_COMMON_DIR.."*.h" } }
        includedirs { FEMFX_INC_DIR, FEMFX_SRC_DIR.."Common/", FEMFX_SRC_DIR.."Bvh/",FEMFX_SRC_DIR.."PrimitiveCollision/", FEMFX_SRC_DIR.."FindContacts/", FEMFX_SRC_DIR.."Scene/", FEMFX_SRC_DIR.."SparseSolvers/", FEMFX_SRC_DIR.."ConstraintSolver/", FEMFX_SRC_DIR.."Simulation/" }
		includedirs { VECTORMATH_INC_DIR, TASK_SYSTEM_INC_DIR, TRACE_DIR, EXAMPLE_RB_DIR, JSON_INC_DIR }
        includedirs { SRC_DIR, SAMPLES_COMMON_DIR }
		libdirs { FEMFX_LIB_DIR }
        links { FEMFX_LIB_NAME, TASK_SYSTEM_NAME, "opengl32", "glu32", "glfw3" }

        filter "action:gmake"
            includedirs { GLFW_GMAKE_BASE_DIR.."include/" }
            libdirs { GLFW_GMAKE_BASE_DIR.."lib/" }
            links { "pthread" }

        filter "action:vs*"
            includedirs { GLFW_VS_INCLUDE_DIR }
            libdirs { GLFW_VS_LIB_DIR }

        filter { "action:vs*", "files:**minitrace.c" }
            disablewarnings {"4201", "4456"}

        filter {}

include "../../amd_femfx"
include "../../amd_femfx/TL"

