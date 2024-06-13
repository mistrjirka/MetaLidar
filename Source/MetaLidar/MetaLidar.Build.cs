// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.IO;
public class MetaLidar : ModuleRules
{
	public MetaLidar(ReadOnlyTargetRules Target) : base(Target)
	{
		//UEBuildConfiguration.bForceEnableExceptions = true;
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		//PublicDependencyModuleNames.Add("PCL");

		PublicIncludePaths.AddRange(
			new string[]
			{}
		);

		PublicAdditionalLibraries.AddRange(
			new string[]
			{}
		);

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				// ... add other public dependencies that you statically link with here ...
			}
			);


		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"PhysicsCore",
				"Projects"
			}
			);


		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
