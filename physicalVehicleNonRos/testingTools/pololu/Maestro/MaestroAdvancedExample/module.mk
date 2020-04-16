# Generate a unique list of files that need to be in the same
# directory as the executable at runtime (runtime dependencies).
MaestroAdvancedExample_runtime := $(sort $(Bytecode_lib) $(UsbWrapper_lib) $(Usc_lib))

# Compile-time dependencies.
MaestroAdvancedExample_dlls := $(Bytecode)/Bytecode.dll $(UsbWrapper)/UsbWrapper.dll $(Usc)/Usc.dll
MaestroAdvancedExample_csfiles := $(MaestroAdvancedExample)/MainWindow.cs $(MaestroAdvancedExample)/MainWindow.Designer.cs $(MaestroAdvancedExample)/Program.cs $(wildcard $(MaestroAdvancedExample)/Properties/*.cs)

# These two resources files don't seem to be needed for compilation, but
# we include them here in case you need them in the future.  This variable
# defines which .resources files are needed.  The rule for making .resources
# files is defined in the Makefile.
MaestroAdvancedExample_resources := \
	$(MaestroAdvancedExample)/MainWindow.resources \
	$(MaestroAdvancedExample)/Properties/Resources.resources

MaestroAdvancedExample_resourceids := \
	Pololu.Usc.MaestroAdvancedExample.MainWindow.resources \
	Pololu.Usc.MaestroAdvancedExample.Properties.Resources.resources

MaestroAdvancedExample_resource_args = $(join \
	$(foreach res, $(MaestroAdvancedExample_resources), -resource:$(res)), \
	$(foreach id, $(MaestroAdvancedExample_resourceids), ,$(id)))

# Required module variables
Targets += $(MaestroAdvancedExample)/MaestroAdvancedExample $(MaestroAdvancedExample_resources)
Byproducts += $(foreach dll, $(MaestroAdvancedExample_runtime), $(MaestroAdvancedExample)/$(notdir $(dll)))

$(MaestroAdvancedExample)/MaestroAdvancedExample: $(MaestroAdvancedExample_csfiles) $(MaestroAdvancedExample_runtime) $(MaestroAdvancedExample_resources)
	cp $(MaestroAdvancedExample_runtime) $(MaestroAdvancedExample)
	$(CS) -target:exe -out:$@.exe $(MaestroAdvancedExample_csfiles) $(foreach dll, $(MaestroAdvancedExample_dlls),-r:$(MaestroAdvancedExample)/$(notdir $(dll))) $(Mono_StandardLibs) $(MaestroAdvancedExample_resource_args)
	mv $@.exe $@

# Alias so you can type "make maestroadvancedexample"
maestroadvancedexample: $(MaestroAdvancedExample)/MaestroAdvancedExample