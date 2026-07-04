#!/usr/bin/env python3
# TVOC from raw ADC (Waveshare PY32 TVOC sensor) -- Pi-side computation.
#
# Why this exists: the sensor firmware computes TVOC = 11.5 - 1.86*ln(501 - R0)
# where R0 is an internal, heavily rank-filtered baseline-resistance value.
# After ~1 year unpowered storage the MOX element drifted so R0 parks near
# 100.3, pinning the firmware TVOC at ~0.352 ppm regardless of air. That value
# is NOT exposed over UART. The raw ADC (query-frame bytes 7,8) IS exposed and
# is still fully responsive (~760 clean air, ~247 under alcohol vapour).
# This script recovers a live, responsive TVOC from that raw ADC.
import serial, time, math, sys
PORT="/dev/ttyS0"; BAUD=115200
QUERY=bytearray([0xFE,0x00,0x86,0x00,0x00,0x00,0x00,0x00,0x86])
# --- tunables ---
C = 1.86            # ppm per ln-unit (borrowed from the firmware curve slope)
BASE_INIT = None    # clean-air ADC baseline; None => auto-calibrate from first samples
def read_adc(ser):
    ser.flushInput(); ser.write(QUERY); time.sleep(0.02)
    d=list(ser.read(11))
    if len(d)<11: return None
    # checksum over bytes 3..8 == byte9 (vendor query framing)
    if (sum(d[3:9]) & 0xFF)!=d[9]: return None
    return d[7]*256+d[8]
def main():
    ser=serial.Serial(PORT,BAUD)
    base=BASE_INIT; warm=[]
    print("# raw-ADC TVOC monitor. baseline auto-calibrates to cleanest air seen.")
    print("#  ADC   ADC0(clean)   ratio   TVOC(ppm, rel)")
    n=int(sys.argv[1]) if len(sys.argv)>1 else 0
    i=0
    while n==0 or i<n:
        adc=read_adc(ser)
        if adc is None:
            time.sleep(0.5); continue
        # baseline = cleanest (highest) ADC observed, with very slow downward creep
        if base is None:
            warm.append(adc)
            if len(warm)>=5: base=max(warm)
            print("  %4d   (calibrating...)"%adc)
        else:
            if adc>base: base=adc                      # cleaner air seen -> raise baseline
            else: base=base*0.99995+adc*0.00005        # ultra-slow drift comp
            ratio=base/max(adc,1)
            tvoc=max(0.0, C*math.log(ratio))
            print("  %4d   %7.1f     %5.3f   %6.3f"%(adc,base,ratio,tvoc))
        i+=1; time.sleep(1.0)
if __name__=="__main__": main()
