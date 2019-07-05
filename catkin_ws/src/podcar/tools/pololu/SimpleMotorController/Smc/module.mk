Targets += $(Smc)/Smc.dll

# Run-time dependencies.
Smc_lib := $(UsbWrapper_lib) $(Smc)/Smc.dll

# Compile-time dependencies.
Smc_csfiles := $(wildcard $(Smc)/*.cs) $(Smc)/Properties/AssemblyInfo.cs
Smc_dlls := $(UsbWrapper)/UsbWrapper.dll

$(Smc)/Smc.dll: $(Smc_dlls) $(Smc_csfiles)
	$(CS) -target:library -out:$@ $(foreach dll, $(Smc_dlls),-r:$(dll)) $(Smc_csfiles)