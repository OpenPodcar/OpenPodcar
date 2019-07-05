# Generate a unique list of files that need to be in the same
# directory as PgmCmd at runtime (runtime dependencies).
PgmCmd_runtime := $(sort $(UsbWrapper_lib) $(Programmer_lib))

# Compile-time dependencies.
PgmCmd_dlls := $(UsbWrapper)/UsbWrapper.dll $(Programmer)/Programmer.dll
PgmCmd_csfiles := $(PgmCmd)/Program.cs $(PgmCmd)/Properties/AssemblyInfo.cs

# Required module variables
Targets += $(PgmCmd)/PgmCmd
Byproducts += $(foreach rt, $(PgmCmd_runtime), $(PgmCmd)/$(notdir $(rt)))

$(PgmCmd)/PgmCmd: $(PgmCmd_csfiles) $(PgmCmd_runtime)
	cp $(PgmCmd_runtime) $(PgmCmd)
	$(CS) -target:exe -out:$@.exe $(PgmCmd_csfiles) $(foreach dll, $(PgmCmd_dlls),-r:$(PgmCmd)/$(notdir $(dll)))
	mv $@.exe $@

# Alias so you can type "make pgmcmd"
pgmcmd: $(PgmCmd)/PgmCmd

