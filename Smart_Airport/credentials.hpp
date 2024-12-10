#ifndef CREDENTIALS_HPP
#define CREDENTIALS_HPP

// Déclaration de la variable pour accéder au Wi-Fi
extern const char* WiFi_ssid;
extern const char* WiFi_Password;

extern const char* mqttServer;
extern const int mqttPort;
extern const char* mqttUser;
extern const char* mqttPassword;

// Les CREDENTIALS ici sont cachés dans un autre dossier appelé credential.cpp
// protégé par le .gitignore

#endif // CREDENTIALS_HPP