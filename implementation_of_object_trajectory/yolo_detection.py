import cv2
from ultralytics import YOLO

def pixel_to_robot_mm(pixel_x, pixel_y, frame_width=640, frame_height=480):
    center_pixel_x = frame_width / 2     
    center_pixel_y = frame_height / 2    
    
    mm_per_pixel_x = 350.0 / frame_width  
    mm_per_pixel_y = 350.0 / frame_height 

    robot_x = (pixel_x - center_pixel_x) * mm_per_pixel_x
    robot_y = (center_pixel_y - pixel_y) * mm_per_pixel_y
    
    return robot_x, robot_y

def get_snapshot():
    """
    Ova funkcija otvara kameru, prikazuje sliku i čeka 'c'.
    Kada korisnik stisne 'c', vraća trenutne lokacije svih objekata.
    """
    model = YOLO("yolov8n.pt") 
    cap = cv2.VideoCapture(0)

    live_object_coordinates = {}
    print("Pričekajte, kamera se učitava... Pritisnite 'c' za snimanje pozicija.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w, _ = frame.shape
        results = model(frame, conf=0.5, verbose=False)
        live_object_coordinates.clear()

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                class_id = int(box.cls[0])
                object_name = model.names[class_id]
                
                robot_x, robot_y = pixel_to_robot_mm(center_x, center_y, frame_width=w, frame_height=h)
                live_object_coordinates[object_name] = (robot_x, robot_y)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        cv2.imshow("YOLO - Pritisni C za pocetak", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            break # Izlazimo iz petlje i vraćamo koordinate
        elif key == ord('q'): 
            live_object_coordinates = None
            break

    cap.release()
    cv2.destroyAllWindows()
    return live_object_coordinates