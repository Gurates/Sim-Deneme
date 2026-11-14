# RSim2 MCP — canlı simülasyon kontrolü

## Yerel MCP bağlantısı

1. RSim2'yi açıp robot projesini yükleyin.
2. Üst araç çubuğunda **MCP > Enable MCP** seçin.
3. **Test Server** ile bağlantıyı doğrulayın. HTTP testi sürümü ve `execute_trajectory` aracını gösterir.
4. **Codex HTTP (TOML)** veya **Claude Code HTTP (JSON)** seçip **Copy Configuration** düğmesine basın.
5. Codex için `config.toml`, Claude Code için MCP JSON ayarındaki `rsim2` kaydını oluşturun. Eski `rsim2` kaydı varsa tamamını yeni kayıtla değiştirin.
6. İstemcide MCP bağlantısını yeniden yükleyin ve `get_robot_snapshot` aracını çağırmasını isteyin.

Örnek istem: “RSim2 sahnesini ve eklem sınırlarını oku. Bir eklemi önce 5 derece ilerletip sonra başlangıç açısına döndüren tek bir `execute_trajectory` görevi çalıştır. Sonucu `get_task_status` ile doğrula.”

HTTP adresi `http://127.0.0.1:38172/mcp` ve erişim anahtarı sabittir.
RSim2 açılıp MCP etkinleştirildiğinde aynı istemci ayarı tekrar kullanılabilir.
Kopyalanan HTTP ayarı yerel kontrol anahtarını içerir. Ayarı kaynak kontrolüne
eklemeyin. HTTP sunucusu yalnızca aynı makinedeki istemcilere açıktır.
HTTP başlıklarını desteklemeyen istemciler için MCP penceresinde stdio seçenekleri vardır.
Stdio istemcisi güncel oturumu yerel keşif dosyasından bulur.

[Codex ayarları](https://learn.chatgpt.com/docs/extend/mcp) · [Claude Code ayarları](https://code.claude.com/docs/en/mcp) · [Streamable HTTP](https://modelcontextprotocol.io/specification/2025-11-25/basic/transports)

## Araçlar

| Araç | İşlev |
|---|---|
| `get_connection_status` | Canlı bağlantı ve hareket izni |
| `get_robot_state` | Eklem kimlikleri, tipleri, açıları ve sınırları (derece), oynatma ve son görev |
| `get_scene` | Düğüm kimlikleri, üst düğümleri ve dünya konumları (sahne birimleri) |
| `get_robot_snapshot` | Eklem, motor, dönüşüm, sahne, çarpışma, hareket ve görev verisini birlikte okur |
| `get_geometry` | Düğüm ve mesh indeksine göre vertex, normal ve indeks dizilerini sayfalı okur |
| `get_motion` | Etkin hareketin ara noktalarını ve oynatma durumunu okur |
| `move_joints` | Tek hedef pozu veya `waypoints` dizisini çalıştırır |
| `execute_trajectory` | Ara noktaları ve varışta kavrama/bırakma olaylarını tek görevde yürütür |
| `move_tcp` | Dünya koordinatlarında TCP konum ve yaklaşma yönü ara noktalarını planlar |
| `capture_object` | Duran gripper ile doğrulanmış temasta nesneyi bağlar |
| `pick_object_top_down` | Üste yaklaşma, gripper yönünü aşağı çevirme, dik iniş, temas doğrulama ve kaldırmayı tek yörüngede yürütür |
| `release_object` | Simülasyonda tutulan nesneyi bulunduğu yerde bırakır |
| `get_task_status` | Son MCP hareketinin kimliği ve durumu |
| `stop_motion` | Mevcut hareketi durdurur, bekleyen AI etkileşimlerini iptal eder |
| `pause_motion`, `resume_motion` | Aynı görevi ve bekleyen olayları koruyarak duraklatır ve sürdürür |

`execute_trajectory` örneği:

```json
{
  "waypoints": [
    {"targets_degrees": {"shoulder": 20, "elbow": 15}},
    {"targets_degrees": {"elbow": 25}},
    {"seconds": 2, "targets_degrees": {"shoulder": 0, "elbow": 0}}
  ]
}
```

Belirtilmeyen eklemler önceki hedefte kalır. `seconds` önceki noktadan
itibaren geçen süredir. Belirtilmezse süre eklem hızı ve ivmesine göre
hesaplanır. `event` alanı için `{"type":"capture","object":"box","tool":"gripper","tool_offset":[0,0,0.05]}`
veya `{"type":"release","object":"box"}` kullanılabilir. Olay ilgili noktaya
varıldığında çalışır. Temas doğrulanamazsa görev `blocked` olur.

`move_joints` örnek argümanı (kimlik gerçek robottan okunmalıdır):

```json
{"targets_degrees":{"shoulder":15}}
```

Hedefler mutlak açılardır; artış miktarı değildir. Hedeflerin tümü uygulanmadan
önce doğrulanır. Kimlikler tam eşleşmelidir; sabit eklemler, sayı olmayan veya
sonlu olmayan hedefler ve limit dışı açılar reddedilir. Robotun eklem kimlikleri
büyük/küçük harf bakımından da benzersiz olmalıdır.

Başlatma yanıtındaki `running` tamamlanma anlamına gelmez. `get_task_status`
ile `completed`, `blocked`, `stopped` veya `interrupted` sonucuna kadar sorgulayın.
Yalnızca en son MCP görevi tutulur. Yeni hareket, çalışan hareketin üzerine
binmez; önce durdurulmalıdır. UI'dan durdurma, sahne değişimi, çarpışma veya
MCP'nin kapatılması görev durumuna yansır. İletişim zaman aşımından sonra
hareketi körlemesine tekrarlamak yerine önce görev durumunu okuyun.

## Mimari ve kapsam

Streamable HTTP istemcisi RSim2 içindeki araç kataloğuna doğrudan bağlanır.
Stdio istemcisi, oturum anahtarı isteyen yerel köprüyü kullanır.
Ağ iş parçacığı simülasyon nesnelerine dokunmaz; kuyruktaki komutlar ana
döngüde yürütülür. Zaman aşımına uğrayan bekleyen komutlar iptal edilir.
Robot durumları da ana döngüde okunur.

Hareket mevcut MotionPlayer/JointMotion ve çarpışma denetiminden geçer.
E-stop etkinken veya Sim2Real live sync/digital twin açıkken MCP hareketi
reddedilir; çalışan MCP hareketinde bunlar açılırsa hareket durur.
Bu sürüm fiziksel robot kontrolünü içermez. Kavrama, temas doğrulandıktan sonra
nesneyi gripper'a kinematik bağlar; parmak kapanma dinamiği ve temas kuvvetleri
simüle edilmez.

## Tek yörüngede üstten kavrama

`pick_object_top_down` tüm aşamaları hareket başlamadan planlar: kutunun üstünde
bekleme noktasına gelme, TCP yaklaşma eksenini dünya -Y yönüne çevirme, dik
iniş, temas doğrulama/bağlama ve yukarı kaldırma. Tek MotionSequence ve tek
görev kimliği kullanılır. İniş ile kaldırma arasındaki temas olayı uzun render
karelerinde bile atlanmaz. Tam yol erişilemiyorsa veya çarpışıyorsa başlangıç
pozu ve nesne hiyerarşisi korunarak hata döner; kol kısmen ilerletilmez.

Örnek (kimlikler ve TCP kalibrasyonu gerçek robottan alınmalıdır):

```json
{
  "object": "box",
  "tool": "gripper",
  "tool_offset": [0, 0, 0.05],
  "tool_axis": [0, 0, 1],
  "clearance": 0.15,
  "lift_distance": 0.1
}
```

`tool_offset` gripper yerel koordinatlarında gerçek temas/TCP noktasıdır;
`tool_axis` aynı yerel koordinatlarda gripper'ın yaklaşma yönüdür. Bunlar
robotun geometrisine bağlıdır; örnekteki değerler evrensel değildir.
`get_scene` dünya eksenlerini, sınırları ve geometriye dayalı bir TCP ipucunu
sunar; bilinmeyen kalibrasyon tahmin edilmemelidir. Döndürme hedefi dik aşağı
yönelimdir: başlangıç yaklaşma yönü yataysa bu 90 derecelik dönüş olur.
Yerel IK çözümü genel erişilebilirlik garantisi vermez; yetersiz serbestlik
derecesi, kalibrasyon, limit veya planlama süresi nedeniyle hata dönebilir.

Görev durumunun `detail` alanı yaklaşma, yönelme, iniş ve kaldırma aşamalarını
gösterir; `capture_verified` gerçek bağlamanın gerçekleştiğini belirtir.
Nesne sonradan taşınırsa, durdurulursa veya döngülü oynatma açılırsa görev
tamamlanmış sayılmaz. `completed` için nesnenin istenen miktarda yükseldiği
ölçülür. Bu olaylı yörüngeyi UI'dan yeniden oynatmak yeni bir kavrama görevi
başlatmaz; tekrar kavramak için nesneyi bırakıp yeni MCP görevi başlatın.

MCP `2025-11-25` ile initialize, ping ve araç listeleme/çağırma desteklenir.
HTTP taşıması durum tutmayan JSON yanıtları kullanır. Protokol sürümü bağlantıda
anlaşılır ve eski istemciler için `2025-06-18` ve `2025-03-26` da desteklenir.
API anahtarı/model API çağrısı gerekmez; AI istemcisinin kendi abonelik ve
kullanım koşulları geçerlidir.

## Derleme

Java 21 ile proje kökünde:

```powershell
.\gradlew.bat :app:installDist
```

Dağıtımda ayrı `rsim2-mcp` başlatıcısı da vardır; önerilen kurulum yolu MCP
penceresidir. Bağımsız sunucu argümansız başlatıldığında varsayılan kullanıcı
keşif dosyasını kullanır. Uygulama kapalıyken bağlantı durumu `false` olur;
uygulama açılıp MCP etkinleştirildiğinde aynı süreç otomatik bağlanır.
Hareket komutları bağlantı hatasından sonra otomatik tekrar gönderilmez.
