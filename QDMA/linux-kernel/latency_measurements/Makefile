CFLAGS = -Ofast -march=native -flto -fno-plt

all: irq_mmr mmr affinity

irq_mmr: irq_mmr.c
	gcc $(CFLAGS) irq_mmr.c -o irq_mmr

mmr: mmr.c
	gcc $(CFLAGS) mmr.c -o mmr

affinity: affinity.c
	gcc $(CFLAGS) affinity.c -o affinity

clean:
	rm -f irq_mmr mmr affinity

realtime_irq_mmr: mmr
	chrt -f 99 ./irq_mmr $(ARGS)
	
realtime_mmr: mmr
	chrt -f 99 ./mmr $(ARGS)

realtime_affinity: affinity
	chrt -f 99 ./affinity $(ARGS)

.PHONY: all clean realtime_mmr realtime_irq_mmr realtime_affinity