# Smartbench - BSP-level Smart Delivery Bench
## Results

### HX711: Noise reduction from 10-sample averaging

The HX711 load-cell driver reads the 24-bit ADC 10 times per measurement and
returns the average, reducing random noise by approximately √10 ≈ 3.16×.

Single-shot quiescent reads show ~326 ADC counts of standard deviation; after
averaging, weighing a 205 g reference (iQOO Neo 7 SE) holds within ~2 g. The
slow downward drift visible in both panels is mechanical creep from a
non-rigid sensor mount, which averaging cannot remove — only rigid mounting can.

![HX711 averaging](docs/fig2_hx711_averaging.png)
