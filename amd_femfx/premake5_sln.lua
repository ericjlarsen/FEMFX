SOLUTION_NAME = "FEMFXLib"
FEMFX_LIB_NAME = "AMD_FEMFX"

solution ( SOLUTION_NAME )

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
    targetdir ( 'build/'.._ACTION..'/%{cfg.architecture}/%{cfg.platform}/%{cfg.buildcfg}' )

    filter "kind:StaticLib"
        defines "_LIB"

    filter "configurations:Debug" 
        defines { "_DEBUG" }
        targetsuffix("_d")

    filter "configurations:Release"
        defines { "NDEBUG" }
        optimize "Speed"

include "."

