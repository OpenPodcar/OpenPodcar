# Generate a unique list of files that need to be in the same
# directory as this program at runtime (runtime dependencies).
SmcCmd_runtime := $(sort $(UsbWrapper_lib) $(Smc_lib))

# Compile-time dependencies.
SmcCmd_dlls := $(UsbWrapper)/UsbWrapper.dll $(Smc)/Smc.dll
SmcCmd_csfiles := $(SmcCmd)/Program.cs $(SmcCmd)/Properties/AssemblyInfo.cs

# Required module variables
Targets += $(SmcCmd)/SmcCmd
Byproducts += $(foreach dll, $(SmcCmd_runtime), $(SmcCmd)/$(notdir $(dll)))

$(SmcCmd)/SmcCmd: $(SmcCmd_csfiles) $(SmcCmd_dlls)
	cp $(SmcCmd_runtime) $(SmcCmd)
	$(CS) -target:exe -out:$@.exe $(SmcCmd_csfiles) $(foreach dll, $(SmcCmd_dlls),-r:$(SmcCmd)/$(notdir $(dll)))
	mv $@.exe $@

# Alias so you can type "make smccmd"
smccmd: $(SmcCmd)/SmcCmd
