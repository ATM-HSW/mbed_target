import d2xx

h = d2xx.open(0)
h.setBitMode(255,32)
h.setBitMode(247,32)
h.setBitMode(255,32)
h.setBitMode(0,32)

h.close()