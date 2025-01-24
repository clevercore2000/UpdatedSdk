package org.firstinspires.ftc.teamcode;

public class Masina
{
    private int roti;
    private int usi;
    private int vit_max;

    Masina()
    {
        roti = 4;
        usi = 4;
        vit_max = 180;
    }
    public void setRoti( int nRoti)
    {
        roti = ( nRoti <=4 && nRoti < 0 )? nRoti : 4;
    }

    public  int getRoti()
    {
        return roti;
    }

}
/*


Masina Mazda = new Masina();

Mazda.set(4);
int cate_roti_are = Mazda.getRoti();

struct Masina
{
    int roti;
    int usi;
    int vit_max;
};

struct Masina Mazda = { 4, 5, 220};

void Initializare()
{
    vit_max = 1000;
    cout << "Roti :" << Mazda.roti;

}






 */