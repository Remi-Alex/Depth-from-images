#include "testCorrespondences.h"

using namespace std;
using namespace cv;

// FOR im1.png and im2.png
void testManualCorrespondencesEpipolar(vector<Point2f>& left, vector<Point2f>& right)
{
    left.push_back(Point2f(158, 232));
    left.push_back(Point2f(310, 285));
    left.push_back(Point2f(158, 226));
    left.push_back(Point2f(150, 331));
    left.push_back(Point2f(197, 317));
    left.push_back(Point2f(303, 274));
    left.push_back(Point2f(160, 325));
    left.push_back(Point2f(159, 138));
    left.push_back(Point2f(235, 341));
    left.push_back(Point2f(241, 262));
    left.push_back(Point2f(257, 271));
    left.push_back(Point2f(194, 291));
    left.push_back(Point2f(188, 330));
    left.push_back(Point2f(203, 212));
    left.push_back(Point2f(252, 285));
    left.push_back(Point2f(161, 277));
    left.push_back(Point2f(221, 294));
    left.push_back(Point2f(133, 330));
    left.push_back(Point2f(157, 306));
    left.push_back(Point2f(475, 385));
    left.push_back(Point2f(265, 171));
    left.push_back(Point2f(149, 274));
    left.push_back(Point2f(302, 358));
    left.push_back(Point2f(151, 221));
    left.push_back(Point2f(153, 226));
    left.push_back(Point2f(254, 284));
    left.push_back(Point2f(195, 160));
    left.push_back(Point2f(448, 394));
    left.push_back(Point2f(162, 246));
    left.push_back(Point2f(191, 353));
    left.push_back(Point2f(289, 292));
    left.push_back(Point2f(399, 339));
    left.push_back(Point2f(361, 274));
    left.push_back(Point2f(232, 342));
    left.push_back(Point2f(491, 154));
    left.push_back(Point2f(256, 215));
    left.push_back(Point2f(265, 281));
    left.push_back(Point2f(267, 232));
    left.push_back(Point2f(479, 104));
    left.push_back(Point2f(163, 139));
    left.push_back(Point2f(201, 154));
    left.push_back(Point2f(195, 270));
    left.push_back(Point2f(160, 332));
    left.push_back(Point2f(146, 231));
    left.push_back(Point2f(379, 284));
    left.push_back(Point2f(227, 311));
    left.push_back(Point2f(148, 132));
    left.push_back(Point2f(154, 135));
    left.push_back(Point2f(238, 260));
    left.push_back(Point2f(224, 311));
    left.push_back(Point2f(410, 236));
    left.push_back(Point2f(197, 190));
    left.push_back(Point2f(422, 237));
    left.push_back(Point2f(55 , 183));
    left.push_back(Point2f(256, 204));
    left.push_back(Point2f(411, 372));
    left.push_back(Point2f(502, 153));
    left.push_back(Point2f(236, 280));
    left.push_back(Point2f(63 , 155));
    left.push_back(Point2f(259, 215));
    left.push_back(Point2f(310, 277));
    left.push_back(Point2f(314, 221));
    left.push_back(Point2f(321, 289));
    left.push_back(Point2f(318, 289));
    left.push_back(Point2f(381, 290));
    left.push_back(Point2f(382, 281));
    left.push_back(Point2f(255, 170));
    left.push_back(Point2f(381, 293));
    left.push_back(Point2f(368, 230));
    left.push_back(Point2f(418, 235));
    left.push_back(Point2f(315, 298));
    left.push_back(Point2f(173, 326));
    left.push_back(Point2f(188, 155));
    left.push_back(Point2f(197, 240));
    left.push_back(Point2f(261, 170));
    left.push_back(Point2f(163, 153));
    left.push_back(Point2f(314, 207));
    left.push_back(Point2f(440, 122));
    left.push_back(Point2f(270, 171));
    left.push_back(Point2f(424, 381));
    left.push_back(Point2f(161, 187));
    left.push_back(Point2f(397, 280));
    left.push_back(Point2f(124, 127));
    left.push_back(Point2f(241, 158));
    left.push_back(Point2f(474, 104));
    left.push_back(Point2f(258, 227));
    left.push_back(Point2f(112, 134));
    left.push_back(Point2f(199, 245));
    left.push_back(Point2f(374, 294));
    left.push_back(Point2f(147, 222));
    left.push_back(Point2f(179, 327));
    left.push_back(Point2f(426, 224));
    left.push_back(Point2f(318, 227));
    left.push_back(Point2f(200, 128));
    left.push_back(Point2f(317, 222));
    left.push_back(Point2f(226, 185));
    left.push_back(Point2f(234, 277));
    left.push_back(Point2f(200, 242));
    left.push_back(Point2f(121, 127));
    left.push_back(Point2f(109, 134));
    left.push_back(Point2f(382, 237));
    left.push_back(Point2f(300, 279));
    left.push_back(Point2f(188, 136));
    left.push_back(Point2f(372, 290));
    left.push_back(Point2f(240, 132));
    left.push_back(Point2f(237, 244));
    left.push_back(Point2f(421, 224));
    left.push_back(Point2f(500, 141));
    left.push_back(Point2f(311, 227));
    left.push_back(Point2f(237, 160));

    right.push_back(Point2f(158, 212));
    right.push_back(Point2f(312, 280));
    right.push_back(Point2f(158, 204));
    right.push_back(Point2f(150, 335));
    right.push_back(Point2f(198, 319));
    right.push_back(Point2f(306, 269));
    right.push_back(Point2f(161, 328));
    right.push_back(Point2f(158, 141));
    right.push_back(Point2f(238, 347));
    right.push_back(Point2f(241, 259));
    right.push_back(Point2f(258, 267));
    right.push_back(Point2f(195, 285));
    right.push_back(Point2f(189, 332));
    right.push_back(Point2f(202, 191));
    right.push_back(Point2f(253, 281));
    right.push_back(Point2f(161, 269));
    right.push_back(Point2f(222, 290));
    right.push_back(Point2f(134, 326));
    right.push_back(Point2f(158, 303));
    right.push_back(Point2f(485, 387));
    right.push_back(Point2f(263, 171));
    right.push_back(Point2f(149, 262));
    right.push_back(Point2f(307, 372));
    right.push_back(Point2f(150, 196));
    right.push_back(Point2f(152, 200));
    right.push_back(Point2f(256, 278));
    right.push_back(Point2f(194, 155));
    right.push_back(Point2f(458, 404));
    right.push_back(Point2f(162, 232));
    right.push_back(Point2f(193, 361));
    right.push_back(Point2f(291, 287));
    right.push_back(Point2f(405, 349));
    right.push_back(Point2f(363, 268));
    right.push_back(Point2f(235, 348));
    right.push_back(Point2f(485, 133));
    right.push_back(Point2f(255, 196));
    right.push_back(Point2f(266, 276));
    right.push_back(Point2f(267, 212));
    right.push_back(Point2f(471, 103));
    right.push_back(Point2f(162, 143));
    right.push_back(Point2f(200, 151));
    right.push_back(Point2f(195, 259));
    right.push_back(Point2f(160, 338));
    right.push_back(Point2f(146, 208));
    right.push_back(Point2f(381, 278));
    right.push_back(Point2f(229, 309));
    right.push_back(Point2f(147, 134));
    right.push_back(Point2f(153, 138));
    right.push_back(Point2f(238, 257));
    right.push_back(Point2f(226, 310));
    right.push_back(Point2f(409, 213));
    right.push_back(Point2f(196, 175));
    right.push_back(Point2f(422, 216));
    right.push_back(Point2f(56 , 176));
    right.push_back(Point2f(255, 188));
    right.push_back(Point2f(418, 386));
    right.push_back(Point2f(495, 131));
    right.push_back(Point2f(237, 271));
    right.push_back(Point2f(64 , 145));
    right.push_back(Point2f(258, 195));
    right.push_back(Point2f(312, 271));
    right.push_back(Point2f(313, 201));
    right.push_back(Point2f(323, 284));
    right.push_back(Point2f(320, 284));
    right.push_back(Point2f(383, 284));
    right.push_back(Point2f(385, 274));
    right.push_back(Point2f(254, 168));
    right.push_back(Point2f(384, 288));
    right.push_back(Point2f(368, 210));
    right.push_back(Point2f(418, 214));
    right.push_back(Point2f(317, 296));
    right.push_back(Point2f(174, 330));
    right.push_back(Point2f(187, 152));
    right.push_back(Point2f(196, 221));
    right.push_back(Point2f(260, 170));
    right.push_back(Point2f(162, 152));
    right.push_back(Point2f(313, 190));
    right.push_back(Point2f(434, 125));
    right.push_back(Point2f(268, 171));
    right.push_back(Point2f(433, 393));
    right.push_back(Point2f(161, 174));
    right.push_back(Point2f(399, 274));
    right.push_back(Point2f(124, 132));
    right.push_back(Point2f(240, 155));
    right.push_back(Point2f(466, 106));
    right.push_back(Point2f(257, 207));
    right.push_back(Point2f(112, 137));
    right.push_back(Point2f(199, 229));
    right.push_back(Point2f(377, 289));
    right.push_back(Point2f(147, 196));
    right.push_back(Point2f(181, 329));
    right.push_back(Point2f(425, 201));
    right.push_back(Point2f(317, 207));
    right.push_back(Point2f(198, 136));
    right.push_back(Point2f(316, 201));
    right.push_back(Point2f(225, 172));
    right.push_back(Point2f(235, 267));
    right.push_back(Point2f(200, 225));
    right.push_back(Point2f(120, 132));
    right.push_back(Point2f(109, 137));
    right.push_back(Point2f(381, 216));
    right.push_back(Point2f(302, 274));
    right.push_back(Point2f(187, 138));
    right.push_back(Point2f(374, 284));
    right.push_back(Point2f(238, 137));
    right.push_back(Point2f(237, 226));
    right.push_back(Point2f(420, 201));
    right.push_back(Point2f(494, 124));
    right.push_back(Point2f(310, 207));
    right.push_back(Point2f(235, 156));
}