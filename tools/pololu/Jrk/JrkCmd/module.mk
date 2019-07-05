# Generate a unique list of files that need to be in the same
# directory as this program at runtime (runtime dependencies).
JrkCmd_runtime := $(sort $(UsbWrapper_lib) $(Jrk_lib))

# Compile-time dependencies.
JrkCmd_dlls := $(UsbWrapper)/UsbWrapper.dll $(Jrk)/Jrk.dll
JrkCmd_csfiles := $(JrkCmd)/Program.cs $(JrkCmd)/Properties/AssemblyInfo.cs

# Required module variables
Targets += $(JrkCmd)/JrkCmd
Byproducts += $(foreach dll, $(JrkCmd_runtime), $(JrkCmd)/$(notdir $(dll)))

$(JrkCmd)/JrkCmd: $(JrkCmd_csfiles) $(JrkCmd_runtime)
	cp $(JrkCmd_runtime) $(JrkCmd)
	$(CS) -target:exe -out:$@.exe $(JrkCmd_csfiles) $(foreach dll, $(JrkCmd_dlls),-r:$(JrkCmd)/$(notdir $(dll)))
	mv $@.exe $@

# Alias so you can type "make jrkcmd"
jrkcmd: $(JrkCmd)/JrkCmd
