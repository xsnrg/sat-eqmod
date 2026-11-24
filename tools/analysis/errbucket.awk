$5 > 0.05 {
    b = int($2 / 1800) * 1800
    d = $4 - $7; if (d > 180) d = 360 - d; if (d < -180) d = 360 + d
    e = sqrt(d * d + ($5 - $8) ^ 2)
    n[b]++; s[b] += e * e
    if (e > mx[b]) { mx[b] = e; mt[b] = $2 }
    if ($21 == 1) halt[b]++
    if ($15 == 1) gl[b]++
}
END {
    for (b in n)
        printf "t=%6d n=%5d rms=%6.2f max=%6.2f@%-7.0f glide=%5d halted=%d\n",
               b, n[b], sqrt(s[b] / n[b]), mx[b], mt[b], gl[b] + 0, halt[b] + 0
}
