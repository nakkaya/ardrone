(ns ardrone.core)

(def drone-ip (java.net.InetAddress/getByName "192.168.1.1"))

(let [counter (atom 0)]
  (defn- cmd-counter []
    (swap! counter inc)))

(let [bb (java.nio.ByteBuffer/allocate 4)
      ib (.asIntBuffer bb)
      fb (.asFloatBuffer bb)]
  (defn- float-to-hex [f]
    (.put fb 0 f)
    (.get ib 0)))

(let [bb (java.nio.ByteBuffer/allocate 4)
      ib (.asIntBuffer bb)
      fb (.asFloatBuffer bb)]
  (defn- hex-to-float [f]
    (.put ib 0 f)
    (.get fb 0)))

(defn- get-int [buff offset]
  (.intValue
   (reduce (fn [h v]
             (let [shift (* v 8)]
               (+ h (bit-shift-left
                     (bit-and (nth buff (+ offset v)) 0xFF)
                     shift))))
           0 (reverse (range 4)))))

(let [socket (agent (doto (java.net.DatagramSocket.)
                      (.setSoTimeout 3000)))]
  (defn- send-at-command [cmd]
    (let [bytes (.getBytes (str cmd "\r"))
          packet (java.net.DatagramPacket. bytes
                                           (count bytes)
                                           drone-ip
                                           5556
                                           )]
      (send socket (fn [s] (try (doto s
                                 (.send packet))
                               (catch Exception e
                                 (println "Warn:" e)
                                 s)))))))

(defonce nav-data-socket (doto (java.net.DatagramSocket. 5554)
                           (.setSoTimeout 5000)))

(defonce nav-data-state (atom {:battery 0
                               :roll    0
                               :yaw     0
                               :alt     0
                               :pitch   0
                               :state :default
                               :listening false}))

(let [trigger-buffer (byte-array (map byte [0x01 0x00 0x00 0x00]))
      trigger-buffer-size (count trigger-buffer)]
  (defn- parse-nav-data []
    (let [packet (java.net.DatagramPacket. trigger-buffer trigger-buffer-size drone-ip 5554)]
      (try
        (.send nav-data-socket packet)
        (let [buffer (byte-array 1024)
              packet-rcv (java.net.DatagramPacket. buffer 1024)]
          (.receive nav-data-socket packet-rcv)

          (if (= (get-int buffer 0) 0x55667788)
            (swap! nav-data-state assoc
                   :battery (get-int buffer 24)
                   :state (let [state (bit-shift-right (get-int buffer 20) 16)]
                            (cond (= state 0) :default
                                  (= state 1) :init
                                  (= state 2) :landed
                                  (= state 3) :flying
                                  (= state 4) :hovering
                                  (= state 5) :test
                                  (= state 6) :trans-takeoff
                                  (= state 7) :trans-gotofix
                                  (= state 8) :trans-landing
                                  :default :invalid))
                   :roll (/ (Float/intBitsToFloat (get-int buffer 32)) 1000)
                   :yaw (/ (Float/intBitsToFloat (get-int buffer 36)) 1000)
                   :alt (double (/ (get-int buffer 40) 1000))
                   :pitch (/ (Float/intBitsToFloat (get-int buffer 40)) 1000))
            (println "Navdata Parse Error")))
        (catch Exception e (println e))))))

;;https://projects.ardrone.org/boards/1/topics/show/68
(defn nav-data-start []
  (if (= (:listening @nav-data-state) false)
    (future
      (swap! nav-data-state assoc :listening true)
      (send-at-command (str "AT*CONFIG=" (cmd-counter) ",\"general:navdata_demo\",\"TRUE\""))
      (while (:listening @nav-data-state)
        (parse-nav-data)
        (Thread/sleep 5))
      (println "Nav Data Listener Exit"))
    (println "Nav Data Listener Running!!")))

(defn nav-data-stop []
  (if (= (:listening @nav-data-state) true)
    (swap! nav-data-state assoc :listening false)
    (println "Nav Data Listener NOT Running!!")))

(defn nav-data []
  @nav-data-state)

;;AT Commands

(defn reset-comm-watchdog []
  (send-at-command (str "AT*COMWDG=" (cmd-counter))))

(defn max-altitude! [alt]
  (send-at-command (str "AT*CONFIG=" (cmd-counter) ",\"control:altitude_max\",\"" alt "\"")))

(defn trim []
  (send-at-command (str "AT*FTRIM=" (cmd-counter))))

(defn takeoff []
  (send-at-command (str "AT*REF=" (cmd-counter) ",290718208")))

(defn land []
  (send-at-command (str "AT*REF=" (cmd-counter) ",290717696")))

(defn reset-emergency []
  (send-at-command (str "AT*REF=" (cmd-counter) ",290717952")))

;;AT*REF=[Sequence number ],[Flag bit-field],[Roll],[Pitch],[Gaz],[Yaw]<LF>

(defonce attitude-state (atom {:yaw 0 :pitch 0 :roll 0 :gaz 0}))

(defn send-attitude-command []
  (let [{:keys [yaw pitch roll gaz]} @attitude-state]
    (send-at-command (str "AT*PCMD=" (cmd-counter) ",1,"
                          (float-to-hex roll) ","
                          (float-to-hex pitch) ","
                          (float-to-hex gaz) ","
                          (float-to-hex yaw)))))

(defn yaw [y]
  (swap! attitude-state assoc :yaw y)
  (send-attitude-command))

(defn pitch [p]
  (swap! attitude-state assoc :pitch p)
  (send-attitude-command))

(defn roll [r]
  (swap! attitude-state assoc :roll r)
  (send-attitude-command))

(defn gaz [g]
  (swap! attitude-state assoc :gaz g)
  (send-attitude-command))

(defn hover []
  (swap! attitude-state assoc :pitch 0 :roll 0 :yaw 0 :gaz 0)
  (send-attitude-command))

(defn attitude
  "Pitch:
    - Negative percentage value will make the drone lower its nose, thus flying frontwards.
    - Positive percentage value will make the drone raise its nose, thus flying backwards.
   Roll:
    - Negative percentage value will make the drone tilt to its left, thus flying leftward.
    - Positive percentage value will make the drone tilt to its right, thus flying rightward.
   Yaw:
    - Negative percentage value will make the drone spin left.
    - Positive percentage value will make the drone spin right.
   Gaz:
    - Negative percentage value will make the drone go down, thus go downwards.
    - Positive percentage value will make the drone rise up in the air, thus go upwards."
  [pitch roll yaw gaz]
  (swap! attitude-state
         (fn [curr]
           (merge-with (fn [a b] (if (nil? b) a b))
                       curr
                       {:pitch pitch :roll roll :yaw yaw :gaz gaz})))
  (send-attitude-command))
