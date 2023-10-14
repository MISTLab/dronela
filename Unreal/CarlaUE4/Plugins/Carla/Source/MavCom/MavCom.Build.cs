// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.
using UnrealBuildTool;
using System;
using System.IO;

public class MavCom : ModuleRules
{
  
  public MavCom(ReadOnlyTargetRules Target) : base(Target)
  {


    bEnableExceptions = true;

    PrivatePCHHeaderFile = "Private/MavComPrivatePCH.h";
    PrivateDependencyModuleNames.AddRange(new string[] {"Core", "CoreUObject", "Engine"}); 
    CppStandard = CppStandardVersion.Cpp17;

    string MavSdkInstallPath = Path.Combine(ModuleDirectory, "../../../../../../MavSdk_LegacyABI/install");
  PublicIncludePaths.AddRange(
    new string[] {
        Path.Combine(MavSdkInstallPath, "include/mavsdk")
    }
  );

  PublicAdditionalLibraries.Add(Path.Combine(MavSdkInstallPath, "lib"));

  RuntimeDependencies.Add(Path.Combine(MavSdkInstallPath, "lib", "libmavsdk.so"));
  
  PublicSystemLibraryPaths.Add("/lib/x86_64-linux-gnu");

  PublicAdditionalLibraries.Add("stdc++");
  PublicAdditionalLibraries.Add("pthread");
  PublicAdditionalLibraries.Add("mavsdk");
 
}
}
