# BFMC_Navigation_System

# 🚗 BFMC Navigation System

Acest proiect face parte din competiția **Bosch Future Mobility Challenge (BFMC)** și implementează un sistem de navigație autonomă pentru un vehicul de dimensiuni reduse. Sistemul folosește algoritmi de planificare a traseului și un model cinematic Ackermann pentru a naviga pe o hartă predefinită.

---

## 📌 **Caracteristici principale**
- **Planificare de traseu:** Utilizează algoritmul *A* pentru a găsi cea mai scurtă rută între punctul de start și destinație.
- **Controlul vehiculului:** Model cinematic Ackermann pentru calculul unghiului de viraj și actualizarea poziției.
- **Execuție paralelă:** Sistemul rulează într-un proces separat folosind `multiprocessing` pentru eficiență.
- **Integrare cu BFMC:** Poate fi utilizat pentru navigarea autonomă în mediul competiției BFMC.

---

## 🛠 **Cum funcționează?**
### 1️⃣ **Generarea traseului**
Programul încarcă un fișier `.graphml` care conține harta circuitului. Apoi, folosește algoritmul *A* pentru a calcula cel mai scurt traseu între start și destinație.

### 2️⃣ **Navigația autonomă**
Pentru fiecare punct al traseului:
- Se calculează unghiul de viraj necesar pentru a ajunge la următorul nod.
- Se actualizează poziția vehiculului pe baza vitezei și a direcției.
- Se verifică dacă vehiculul a ajuns la destinație.

### 3️⃣ **Execuție în proces paralel**
Sistemul de navigație este gestionat prin `multiprocessing`, rulând într-un proces separat pentru a evita blocajele și a permite integrarea cu alte module.

