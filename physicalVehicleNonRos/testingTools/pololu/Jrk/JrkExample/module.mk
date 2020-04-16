# Generate a unique list of files that need to be in the same
# directory as the executable at runtime (runtime dependencies).
JrkExample_runtime := $(sort $(UsbWrapper_lib) $(Jrk_lib))

# Compile-time dependencies.
JrkExample_dlls := $(UsbWrapper)/UsbWrapper.dll $(Jrk)/Jrk.dll
JrkExample_csfiles := $(JrkExample)/MainWindow.cs $(JrkExample)/MainWindow.Designer.cs $(JrkExample)/Program.cs $(wildcard $(JrkExample)/Properties/*.cs)

# These two resources files don't seem to be needed for compilation, but
# we include them here in case you need them in the future.  This variable
# defines which .resources files are needed.  The rule for making .resources
# files is defined in the Makefile.
JrkExample_resources := \
	$(JrkExample)/MainWindow.resources \
	$(JrkExample)/Properties/Resources.resources

JrkExample_resourceids := \
	Pololu.Jrk.JrkExample.MainWindow.resources \
	Pololu.Jrk.JrkExample.Properties.Resources.resources

JrkExample_resource_args = $(join \
	$(foreach res, $(JrkExample_resources), -resource:$(res)), \
	$(foreach id, $(JrkExample_resourceids), ,$(id)))

# Required module variables
Targets += $(JrkExample)/JrkExample $(JrkExample_resources)
Byproducts += $(foreach dll, $(JrkExample_runtime), $(JrkExample)/$(notdir $(dll)))

JrkExample_StandardLibs := $(Mono_StandardLibs)

$(JrkExample)/JrkExample: $(JrkExample_csfiles) $(JrkExample_runtime) $(JrkExample_resources)
	cp $(JrkExample_runtime) $(JrkExample)
	$(CS) -target:exe -out:$@.exe $(JrkExample_csfiles) $(foreach dll, $(JrkExample_dlls),-r:$(JrkExample)/$(notdir $(dll))) $(JrkExample_StandardLibs) $(JrkExample_resource_args)
	mv $@.exe $@

# Alias so you can type "make jrkexample"
jrkexample: $(JrkExample)/JrkExample
