# MQTT TLS

[← Documentation index](README.md)

TLS support or MQTT is currently implemented only for the ESP32 version. TLS is disabled by default and can be enabled in the settings. When enabling, make sure that your MQTT broker accepts secure connection from heishamon, including setting up a PEM CA certificate (e.g. self-signed). Such a certificate will roughly look as follows (the x's being seemingly random characters... the BEGIN and END lines belong to the certificate):
-----BEGIN CERTIFICATE-----
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
===========THIS=====IS====JUST===A====PLACEHOLDER===============================
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
-----END CERTIFICATE-----
Upolad a text file (plain text with suffix pem, e.g. "CA.pem") of the certificate to heishamon in the setting screen. If the syntaxt is correct it will be stored, otherwise discarded. After having uploaded the certificate, activate TLS support and (in usual setups) switch the MQTT port to 8883. !!There will be no support for TLS problems!! Google can help you with problems. In case of problems, first try without TLS support and only after that works try to switch TLS support on.
Currently, there is no possibility to delete a once-uploaded CA certificate. So if you want to remove your certificate, simply upload the dummy/sample above.
