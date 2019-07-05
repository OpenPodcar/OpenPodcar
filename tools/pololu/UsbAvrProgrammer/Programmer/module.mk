Programmer_lib := $(UsbWrapper_lib) $(Programmer)/Programmer.dll
Targets += $(Programmer)/Programmer.dll

Programmer_csfiles := $(Programmer)/Programmer.cs $(Programmer)/ProgrammerSettings.cs

Programmer_dlls := $(UsbWrapper)/UsbWrapper.dll

$(Programmer)/Programmer.dll: $(Programmer_dlls) $(Programmer_csfiles)
	$(CS) -target:library -out:$@ $(foreach dll, $(Programmer_dlls),-r:$(dll)) $(Programmer_csfiles)