; WARNING: The boot rom checks the output of this program byte for byte. So if you
; mess with it, you need to change that too! Better plan is don't mess with it.

; This is the flash stub used by JCP to boot the Skunkboard into flash mode
; All it does is load the vector from the vector table and jump to it

	; store the filesize in RAM for block erase
	; if you change this code you need to change JCP since it
	; manually patches this value
	move.l	#$0ABCDEF0,d1
	; write it twice, once xor'd, so the flash code knows it's valid
	move.l	d1,$3ff0
	eor.l	#$ffff,d1
	move.l	d1,$3ff4

	; address of the flasher vector is $800804
	move.l	$800804,a1
	jmp (a1)
	
; yeah, that's it.

; building this one manually since it's so simple
; mac -fb flashstub.s
; aln -w -e -v -rd -a 4100 x 4000 -o flashstub.cof flashstub.o

