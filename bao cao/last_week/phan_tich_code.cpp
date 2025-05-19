//tap trung chinh o ham xu ly viewpoint hoac ham phan chia vung xem cai nao on hon

exploration_planning.h/cpp
//hai ham chinh de xu ly Viewpoint 
ham dung chinh de xu ly viewpoint la assessUnknownRegion (xu ly cac thong so cua viewpoint) va purgeViewpoint (loc viewpoint tot nhat) thong qua ba thong so so luong luoi di qua (disgridnum), do an toan (travel_degree), chi phi duong di (rv_path_cost)
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

ultis.h/cpp: cung cap ham tien ich de ho tro xac dinh viewpoint 
getAngleRobot: tinh goc giua robot va 1 diem cu the, huu ich de danh gia huong cua viewpoint
Bresenham: thuat toan ve duong thang kiem tra tu viewpoint den centroid co va vao o nao co chuong ngai vat khong (thuat toan con trong disgridnum)

path_planning.h/cpp: quan ly viec lap ke hoach duong di tu vi tri hien tai cua robot den cac viewpoint:
setViewPointset: luu tru cac diem Viewpoint 
setCentroidPaiRiIndex: xac dinh chi so cua cac centroids 
getExplorationPath: su dung thuat toan tim kiem do thi (GraphSearch) de tao duog di tu robot den viewpoint hoac centroids

exploration_planning.cpp: 
getRouterOrder su dung thuat toan Two-Opt de toi uu hoa thu tu tham Centroids, can nhac giua chi phi duong di (rv_path_cost) va ham loi ich thong tin (disgridnum)
ham excute chay dinh ki:
Cập nhật vị trí robot (getRobotPosition).
Tính toán lại centroids và viewpoints nếu cần (getCentroid).
Chọn cặp centroids tối ưu (CandiCentroidPair) và các viewpoints tương ứng.
Gửi các viewpoints và centroids được chọn đến path_planner_ để lập kế hoạch đường đi.
Xuất bản đường đi khám phá (exporation_path) và tuyến đường tổng thể (RoutePath) qua các topic ROS.
ham go_home tao dieu kien de quay lai, tuy nhien chua dua ra viec quay lai nhu nao

path_planning.cpp:
getExplorationPath tao duong di tu robot den 1 viewpoint hoac centroid bang cach 
 - thiet lap diem dau cuoi 
 - su dung thuat toan tim kiem do thi
 - neu khong tim duoc duong di truc tiep, su dung thuat toan tim duong lan can (zd_x16) va (zd_y16) de tim duong thay thev 
 - ham finePATH tinh chinh duong di khi cap nhat vi tri 
 - ham getPositionPath chuyen doi duong di tu toa do luoi sang toa do lien tuc (khong quan trong den thuat toan lam )

 //dung o doan thong so traver_degree
Nguồn gốc của traver_degree: được lấy trực tiếp từ giá trị occupancy của ô lưới tại vị trí viewpoint trong bản đồ global_map_ (một nav_msgs::OccupancyGrid):
choosecentroid->viewpoint_fir[i].traver_degree 
    = global_map_.data[choosecentroid->viewpoint_fir[i].indexX + choosecentroid->viewpoint_fir[i].indexY * width_];
gia tri occupancy duoc danh gia nhu sau 
    -1: Vùng chưa biết (unknown).
    0: Hoàn toàn rảnh rỗi (free). an toan de di chuyen
    100: Hoàn toàn bị chiếm dụng (occupied). do an toan sieu thap vi vung do da bi chiem dung hoan toan 
    s_2 = max_traver_degree > 0 ? float(choosecentroid->viewpoint_fin[j].traver_degree / max_traver_degree) : 0;
    choosecentroid->viewpoint_fin[j].score = 1 * s_1 + s_2 + s_3; 
    chi lay thong so s2 voi dieu kien >0 
    

    bool getExplorationPath(Viewpoint& viewpoint, nav_msgs::Path& path) {
        bool success = planner_->Search(start, goal, path);
        if (!success) {
            ROS_WARN("Failed to find path to viewpoint (%d, %d). Triggering replanning.", viewpoint.indexX, viewpoint.indexY);
            viewpoint.valid = false; // Đánh dấu viewpoint không khả thi
            return false;
        }
        return true;
    }
    if (!path_planner_.getExplorationPath(current_viewpoint, exploration_path)) {
    // Chọn viewpoint tiếp theo từ evaluated_viewpoints
    if (!evaluated_viewpoints.empty()) {
        current_viewpoint = evaluated_viewpoints.begin()->second;
        evaluated_viewpoints.erase(evaluated_viewpoints.begin());
    } else {
        ROS_WARN("No more valid viewpoints. Switching to go_home mode.");
        go_home_ = true;
    }
}

