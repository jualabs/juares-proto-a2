% parser utilizado na 
fun
(#{port := PORT}, <<LAT:32/big-signed-integer, LNG:32/big-signed-integer, NSAT:8, HDOP:16, ALT:16/big-signed-integer>>) when PORT == 4 ->
    #{lat => LAT/1000000, lng => LNG/1000000, nsat => NSAT, hdop => HDOP, alt => ALT};
(#{port := PORT}, <<VBAT:16>>) when PORT == 8 ->
    #{vbat => VBAT}
end.

