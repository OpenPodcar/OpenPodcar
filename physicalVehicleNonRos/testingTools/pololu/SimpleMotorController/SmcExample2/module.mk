# Generate a unique list of files that need to be in the same
# directory as the executable at runtime (runtime dependencies).
SmcExample2_runtime := $(sort $(UsbWrapper_lib) $(Smc_lib))

# Compile-time dependencies.
SmcExample2_dlls := $(UsbWrapper)/UsbWrapper.dll $(Smc)/Smc.dll
SmcExample2_csfiles := $(SmcExample2)/MainWindow.cs $(SmcExample2)/MainWindow.Designer.cs $(SmcExample2)/Program.cs $(wildcard $(SmcExample2)/Properties/*.cs)

# This variable defines which .resources files are needed.  The
# rule for making .resources files is defined in the Makefile.
SmcExample2_resources := \
	$(SmcExample2)/MainWindow.resources \
	$(SmcExample2)/Properties/Resources.resources
SmcExample2_resourceids := \
	Pololu.SimpleMotorController.SmcExample2.MainWindow.resources \
	Pololu.SimpleMotorController.SmcExample2.Properties.Resources.resources
SmcExample2_resource_args = $(join \
	$(foreach res, $(SmcExample2_resources), -resource:$(res)), \
	$(foreach id, $(SmcExample2_resourceids), ,$(id)))

# Required module variables
Targets += $(SmcExample2)/SmcExample2 $(SmcExample2_resources)
Byproducts += $(foreach dll, $(SmcExample2_runtime), $(SmcExample2)/$(notdir $(dll)))

$(SmcExample2)/SmcExample2: $(SmcExample2_csfiles) $(SmcExample2_runtime) $(SmcExample2_resources)
	cp $(SmcExample2_runtime) $(SmcExample2)
	$(CS) -target:exe -out:$@.exe $(SmcExample2_csfiles) $(foreach dll, $(SmcExample2_dlls),-r:$(SmcExample2)/$(notdir $(dll))) $(Mono_StandardLibs) $(SmcExample2_resource_args)
	mv $@.exe $@

# Alias so you can type "make smcexample2"
smcexample2: $(SmcExample2)/SmcExample2