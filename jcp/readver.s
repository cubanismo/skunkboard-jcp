; quickie tool to dump the skunkboard rev and serial into a JCP buffer
; intended for use from JCP
; we need this because there's no 100% reliable way to read the version
; of a Skunkboard from the PC, only the latest versions place it in
; USB memory. This is used for the firmware updater, so we want it
; to cover all cases. It's just a copy of the code used in the bios.
; mac -fb readver.s
; aln -w -e -v -rd -a 5000 x 4000 -o readver.cof readver.o

;; Like the ROM Dump, this code is really, really dumb, expecting 
;; to run from the skunkboard startup. It won't even init hardware
;; or set up the OPL, since that's all technically done.
;; This one doesn't even use the skunk console.

	.include "jaguar.inc"

start:
	move.l	#$800000, a3		; a3 = HPI write data
	move.l	#$C00000, a5		; a5 = HPI write address, read data

	; copy the board's revision and serial number into the
	; bottom of the first buffer. Tag is $FA57F00D
	; This way JCP can get the data.
	move.w	#$4BA0, (a5)		; Destined For BAnk 0!
	move.w	#$4001, (a5)		; Flash read-only mode
	move.l	$800800, d0			; get board version
	move.l	$800808, d1			; get serial number
		
	move.w	#$4004, (a5)	 	; HPI write mode
	move.w	#$2800,	(a5)		; $2800 buffer
	move.w	#$fa57, (a3)
	move.w	#$f00d, (a3)
	move.w	d0, (a3)			; half of version
	swap	d0
	move.w	d0, (a3)			; other half
	move.w	d1, (a3)			; half of serial
	swap	d1
	move.w	d1, (a3)			; other half

	; check how to return (depends on BIOS rev)
	move.l	$800800, d0
	cmp.l	#$00010100,d0
	beq.s	needjmp
	cmp.l	#$00010000,d0
	beq.s	needjmp
	
	; should be safe to rts - quick and clean return to command mode
	move.w  #$0000,BORD1       	; black border (shows if hangs)
	rts

needjmp:
	; fake a reset by rebooting the cart. This is not much
	; slower (visibly), but it does cause a USB reset.
	move.l $800404,a0
	jmp (a0)
	
	.end
