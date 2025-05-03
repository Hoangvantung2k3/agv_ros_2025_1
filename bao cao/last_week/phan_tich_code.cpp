ham dung chinh de xu ly viewpoint la assessUnknownRegion (xu ly cac thong so cua viewpoint) va purgeViewpoint (loc viewpoint tot nhat)
dinh vi toa do tu ham getCentroid va getCentroidHelper sau do moi danh gia viewpoint tu cac ham assessUnknowRegion va purgeViewpoint





getCentroid và getCentroidHelper tạo ra danh sách các centroids và viewpoints ban đầu dựa trên bản đồ occupancy grid, tập trung vào các ô chưa biết gần ranh giới.
// phan_tich_code
getCentroid (exploration_planning.cpp):
Hàm này chịu trách nhiệm xác định các centroids (trung tâm của các vùng chưa biết) trong một cửa sổ khám phá (ExactWindow). Nó chia cửa sổ thành các cửa sổ con (sub-windows) và gọi getCentroidHelper để xử lý từng cửa sổ con.
Mục tiêu là tìm các khu vực có số lượng ô chưa biết (unknownnum) đủ lớn (vượt qua ngưỡng unknown_num_thre_, mặc định là 80) để coi là vùng đáng khám phá.
Hàm này cũng tạo các visual markers (như windows_marker_) để hiển thị ranh giới của các cửa sổ trên RViz.
getCentroidHelper (exploration_planning.cpp):
Hàm này quét qua các ô trong một cửa sổ con (kích thước w x h, bắt đầu từ tọa độ ori_x, ori_y) để xác định các ô chưa biết (global_map_.data[i + j * width_] == -1).
Đối với mỗi ô chưa biết, nó kiểm tra các ô lân cận (dùng d_x8, d_y8 cho 8 hướng) để đảm bảo ô đó nằm gần ranh giới giữa vùng đã biết và chưa biết (điều kiện count_thre == 3 || count_thre == 4).
Nếu ô thỏa mãn, nó được coi là một viewpoint tiềm năng và được thêm vào danh sách viewpoint_fir của centroid tương ứng.
Hàm cũng tính trung bình tọa độ của các ô chưa biết để xác định vị trí của centroid (centroidposition).

// noi chung la xac dinh duoc thang nao la viewpoint o thoi diem ban dau, dua vao viec no o vung unknonw, sau do dung ham getCentroidHelper de xac dinh no co thoa man nam o vung bien gioi hay khong, thi khong can quan tam nhieu vi phan loc o assessUnknownRegion