# Generate a unique list of files that need to be in the same
# directory as the executable at runtime (runtime dependencies).
SmcExample1_runtime := $(sort $(UsbWrapper_lib) $(Smc_lib))

# Compile-time dependencies.
SmcExample1_dlls := $(UsbWrapper)/UsbWrapper.dll $(Smc)/Smc.dll
SmcExample1_csfiles := $(SmcExample1)/MainWindow.cs $(SmcExample1)/MainWindow.Designer.cs $(SmcExample1)/Program.cs $(wildcard $(SmcExample1)/Properties/*.cs)

# This variable defines which .resources files are needed.  The
# rule for making .resources files is defined in the Makefile.
SmcExample1_resources := \
	$(SmcExample1)/MainWindow.resources \
	$(SmcExample1)/Properties/Resources.resources
SmcExample1_resourceids := \
	Pololu.SimpleMotorController.SmcExample1.MainWindow.resources \
	Pololu.SimpleMotorController.SmcExample1.Properties.Resources.resources
SmcExample1_resource_args = $(join \
	$(foreach res, $(SmcExample1_resources), -resource:$(res)), \
	$(foreach id, $(SmcExample1_resourceids), ,$(id)))

# Required module variables
Targets += $(SmcExample1)/SmcExample1 $(SmcExample1_resources)
Byproducts += $(foreach dll, $(SmcExample1_runtime), $(SmcExample1)/$(notdir $(dll)))

$(SmcExample1)/SmcExample1: $(SmcExample1_csfiles) $(SmcExample1_runtime) $(SmcExample1_resources)
	cp $(SmcExample1_runtime) $(SmcExample1)
	$(CS) -target:exe -out:$@.exe $(SmcExample1_csfiles) $(foreach dll, $(SmcExample1_dlls),-r:$(SmcExample1)/$(notdir $(dll))) $(Mono_StandardLibs) $(SmcExample1_resource_args)
	mv $@.exe $@

# Alias so you can type "make smcexample1"
smcexample1: $(SmcExample1)/SmcExample1