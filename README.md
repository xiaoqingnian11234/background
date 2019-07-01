# background
A Progressive Background Updating Based Coding Scheme for Surveillance Videos
Background based coding is an effective scheme to improve the coding efficiency of surveillance videos. However,
it takes a long time to generate a high quality background picture (BG-picture). And the encoding of the high quality BGpicture
will increase the bitrate abruptly. To solve these problems, a progressive background updating based coding
scheme is proposed in this paper. In the proposed scheme, the BG-picture is updated block by block. To improve the overall
coding efficiency, an importance map is designed to select the valid background blocks (B-blocks) progressively which will be
encoded with high quality. It is worth noting that only the valid B-blocks is encoded instead of the entire BG-picture. Compared
with the reference software of Versatile Video Coding (VVC), the proposed scheme achieves about 23.3 percent bit-rate saving on average.


Sequences
The Proposed method vs. VTM1.1
BD-rate (Y,U,V)
BasketballDrill -38.62 -40.29 -38.68
SYDLNoon -25.14 -41.69 -35.32
WDGChangNoon -28.52 -42.88 -44.94
BWFenMor -31.94 -40.39 -39.78
FourPeople -20.71 -34.78 -30.21
Johnny -14.42 -37.24 -27.99
KristenAndSara -19.48 -35.63 -31.36
Vidyo1 -14.36 -20.38 -22.83
Vidyo3 -19.45 -20.34 -51.21
Vidyo4 -20.66 -32.45 -37.80
Average -23.33 -34.61 -36.01
