Jrk_lib := $(UsbWrapper_lib) $(Jrk)/Jrk.dll
Targets += $(Jrk)/Jrk.dll

Jrk_csfiles := $(wildcard $(Jrk)/*.cs)

Jrk_dlls := $(UsbWrapper)/UsbWrapper.dll

$(Jrk)/Jrk.dll: $(Jrk_dlls) $(Jrk_csfiles)
	$(CS) -target:library -out:$@ $(foreach dll, $(Jrk_dlls),-r:$(dll)) $(Jrk_csfiles)