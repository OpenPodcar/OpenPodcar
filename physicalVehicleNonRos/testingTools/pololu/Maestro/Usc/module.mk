Usc_lib := $(UsbWrapper_lib) $(Bytecode_lib) $(Sequencer_lib) $(Usc)/Usc.dll
Targets += $(Usc)/Usc.dll

Usc_csfiles := $(Usc)/ConfigurationFile.cs \
  $(Usc)/IUscSettingsHolder.cs \
  $(Usc)/Usc.cs \
  $(Usc)/Usc_protocol.cs \
  $(Usc)/UscSettings.cs

Usc_dlls := $(Bytecode)/Bytecode.dll $(UsbWrapper)/UsbWrapper.dll $(Sequencer)/Sequencer.dll


$(Usc)/Usc.dll: $(Usc_dlls) $(Usc_csfiles)
	$(CS) -target:library -out:$@ $(foreach dll, $(Usc_dlls),-r:$(dll)) $(Usc_csfiles)