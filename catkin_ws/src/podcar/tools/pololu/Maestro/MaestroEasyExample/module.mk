# Generate a unique list of files that need to be in the same
# directory as the executable at runtime (runtime dependencies).
MaestroEasyExample_runtime := $(sort $(UsbWrapper_lib) $(Usc_lib))

# Compile-time dependencies.
MaestroEasyExample_dlls := $(UsbWrapper)/UsbWrapper.dll $(Usc)/Usc.dll
MaestroEasyExample_csfiles := $(MaestroEasyExample)/MainWindow.cs $(MaestroEasyExample)/MainWindow.Designer.cs $(MaestroEasyExample)/Program.cs $(wildcard $(MaestroEasyExample)/Properties/*.cs)

# This variable defines which .resources files are needed.  The
# rule for making .resources files is defined in the Makefile.
MaestroEasyExample_resources := \
	$(MaestroEasyExample)/MainWindow.resources \
	$(MaestroEasyExample)/Properties/Resources.resources
MaestroEasyExample_resourceids := \
	Pololu.SimpleMotorController.MaestroEasyExample.MainWindow.resources \
	Pololu.SimpleMotorController.MaestroEasyExample.Properties.Resources.resources
MaestroEasyExample_resource_args = $(join \
	$(foreach res, $(MaestroEasyExample_resources), -resource:$(res)), \
	$(foreach id, $(MaestroEasyExample_resourceids), ,$(id)))

# Required module variables
Targets += $(MaestroEasyExample)/MaestroEasyExample $(MaestroEasyExample_resources)
Byproducts += $(foreach dll, $(MaestroEasyExample_runtime), $(MaestroEasyExample)/$(notdir $(dll)))

$(MaestroEasyExample)/MaestroEasyExample: $(MaestroEasyExample_csfiles) $(MaestroEasyExample_runtime) $(MaestroEasyExample_resources)
	cp $(MaestroEasyExample_runtime) $(MaestroEasyExample)
	$(CS) -target:exe -out:$@.exe $(MaestroEasyExample_csfiles) $(foreach dll, $(MaestroEasyExample_dlls),-r:$(MaestroEasyExample)/$(notdir $(dll))) $(Mono_StandardLibs) $(MaestroEasyExample_resource_args)
	mv $@.exe $@

# Alias so you can type "make maestroeasyexample"
maestroeasyexample: $(MaestroEasyExample)/MaestroEasyExample