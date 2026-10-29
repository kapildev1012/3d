function[output,nodal_positions] = forward_tensegrity(del_sf ,F_inp )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%	Parameters / Parametereingabe %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

aufl2=20;                  % No. of load increments / Anzahl Lastinkremente
aufl=10;                   % No. of iterations / Anzahl Iterationen

kraftkriterium=1e-6;                % Force convergence criterium / Kraftkonvergenzkriterium                  [N]
verschiebungskriterium=1e-6;        % Displacement convergence criterium / Verschiebungskonvergenzkriterium   [mm]

del_s1= del_sf(1);  del_s2= del_sf(2);  del_s3= del_sf(3);  del_f1= del_sf(4); del_f2= del_sf(5);

k_s1= 1e6;    L_s1= del_s1*1      ; % stiffness [N/mm] and rest length [mm] of strut 1
k_s2= 1e6;    L_s2= del_s2*0.4    ; % stiffness [N/mm] and rest length [mm] of strut 2
k_s3=1e6;     L_s3=del_s3* 0.5    ; % stiffness [N/mm] and rest length [mm] of strut 3
k_f1=del_f1* 0.2;    L_f1=0.2    ; % stiffness [N/mm] and rest length [mm] of spring type 1
k_f2=del_f2* 0.3;    L_f2=0.2    ; % stiffness [N/mm] and rest length [mm] of spring type 2

%%%%%%%%%%%%%%%%%%%%%%%%%%% external force / aussere Kraft %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Kraftvonaussen=[  % externe Kraftbelastung an den Knoten in x-, y- und z-Richtung [N] (1./2./3./4. Spalte: x/y/z/KnotenNr.)
       % -0.04 0.01	0   6   % Force on node 6 with components: Fx=-0.04 N ; Fy=0.01 N
       F_inp
];

%%%%%%%%%%%%%%%%%%%%%%%%%%% nodal coordinates / Knotenkoordinaten %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nkoord=[0 0         0       % nodal coordinates in mm
        1 0         0
        0.2  -0.4   0
        0.3   0.2   0
        0.8  -0.3   0
        0.6   0.4   0
        ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%	geometric boundary conditions / Vektor Knotenfreiheitsgrad %%%%
fest=ones(size(nkoord));        % Def. geom. BC. / Vordefinition der Lagerungsbedingungen; (1: movable(beweglich), 0: fixed(nicht beweglich))

fest(1,1:3)=0;                  % node / Knoten 1: fix (fixed in x,y,z)
fest(2,2:3)=0;                  % node / Knoten 2: Loslager (fixed in y,z)
fest(:,3)=0;                    % planar / Ebene Betrachtung (all nodes are fixed in z)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%	Connectivity / Matrix der Knotenkonnektivitaet und Elementparameter 

% Elementmatrix (1,2. Spalte: Knoten der Elementes - % KnotNr. Spalte 1 < Knotennr. Spalte 2 (Knoten fortlaufend nummeriert)
%               (1,2. column: Nodes of the element - % Node Nr. column 1 < Node Nr. column 2 (nodes numbered consecutively)   

elmatrix=[                                
	1 2 k_s1    L_s1
	3 4 k_s2    L_s2 
	5 6 k_s3    L_s3
	1 4 k_f1    L_f1   
	1 3 k_f1    L_f1
	4 6 k_f2    L_f2 % row 6: the Element No. 6 is between nodes 4 and 6 with stiffness k_f2 and rest length L_f2
	3 5 k_f1    L_f1
	2 6 k_f1    L_f1
    2 5 k_f1    L_f1
];

nummerierung=1:length(elmatrix(:,1));
elmatrix=[nummerierung' elmatrix]; % El. numbering in first column / in die 1. Spalte El.Nr. einfuegen
%





%%%%%%%%%%	Reshaping of Force Vector: Vektor der aeusseren Knotenkraefte %
Fext=zeros(size(nkoord,1),3);	
for i=1:size(Kraftvonaussen,1); Fext(Kraftvonaussen(i,4),:)=Kraftvonaussen(i,1:3); end
Fext_vek=reshape(Fext',[numel(Fext),1]);

% %%%%%%%%%%%%%%%% BILD 0 PLOTTEN - Initialkonfiguration  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% C='Color'; BCOL='BackgroundColor'; FN='FontName'; FS='fontsize';
% UN='Units'; PUN='PaperUnits'; PS='PaperSize'; CM='centimeters'; POS='Position'; PPOS='PaperPosition';  
% XACH='XAxisLocation'; YACH='YAxisLocation'; ZACH='ZAxisLocation'; GLS='GridLineStyle';
% LW='Linewidth'; MS='MarkerSize'; MFC='MarkerFaceColor'; MEC='MarkerEdgeColor'; 
% HA='HorizontalAlignment'; VA='VerticalAlignment'; ROT='Rotation';
%     fsiz=6; tim='Calibri'; lwgca=0.5; gls='-.'     						; % Schriftgroesse innerhalb des Diagrammes, gca Linienbreite
%     %lws=4; lws2=4; lwf=2; lwf2=2; schwarz=[0 0 0];  					; % Darstellungsparameter
%     blau=[0 0.4470 0.7410]; rot='#e61919'; gelb=[0.9290 0.6940 0.1250]; gruen='#00e100'; hgrau=[0.7 0.7 0.7];
% 
% xlage=4; ylage=2                                     							                ; % Position auf Bildschirm
% ho=10; ve=10; xan=0.125; yan=0.125; xen=0.025; yen=0.025  						                ; % Bildgroesse, Diagrammgroesse innerhalb des Bildes
% figure(UN,CM,POS,[xlage,ylage,ho,ve],C,'w')                                               		; % Bildlage
% set(gca,UN,CM,POS,[xan*ho yan*ve (1-xen-xan)*ho (1-yen-yan)*ve],FN,tim,FS,fsiz,C,'w')     		; % Diagrammposition, Schrift
% set(gca,'XScale','linear','XDir','normal'); set(gca,XACH,'bottom',YACH,'left')             		; % Achsenskalierung, Achsenlage
% set(gcf,PUN,CM,PS,[ho ve],PPOS,[0 0 ho ve],C,'w'); set(gca,LW,lwgca,GLS,gls)              		; % Papiergroesse, Rahmenfarbe, Gridlinestyle
% 
% xan=1; yan=1; xen=0.25; yen=0.25									                            ;
% subplot(POS,[xan/ho yan/ve (ho-xan-xen)/ho (ve-yen-yan)/ve],FN,tim,FS,fsiz,C,'w'); hold on		;
% set(gca,FN,tim,FS,fsiz,C,'w');  
% 
% plot(nkoord(:,1),nkoord(:,2),'ko',MS,2,LW,2); hold on;
% for i=1:size(nkoord(:,1)); text(nkoord(i,1)+0.02,nkoord(i,2),num2str(i),FN,tim,FS,fsiz);  end
% 
%     for i=1:size(elmatrix,1); hold on
%         if elmatrix(i,4)==k_s1; Farb='k-' ; lw=2; end
%         if elmatrix(i,4)==k_s2; Farb='k-' ; lw=2; end
%         if elmatrix(i,4)==k_s3; Farb='k-' ; lw=2; end
%         if elmatrix(i,4)==k_f1; Farb='b-' ; lw=1; end
%         if elmatrix(i,4)==k_f2; Farb='r-' ; lw=1; end
% 
% 	           plot3([nkoord(elmatrix(i,3),1) nkoord(elmatrix(i,2),1)],...				; % plot Initialkonfiguration
% 	                 [nkoord(elmatrix(i,3),2) nkoord(elmatrix(i,2),2)],...				;
% 	                 [nkoord(elmatrix(i,3),3) nkoord(elmatrix(i,2),3)],Farb,LW,lw)		;
% 
%     end
% 
% xlim([-0.1,1.1]); ylim([-0.6,0.6]);
% box on; axis 'equal'

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Form finding without external forces / Formfindung
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic
%%%%%%%%%% Initialisierung %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k_vek       =elmatrix(:,4)'	                                                    ; % Vektor der Federsteifigkeiten
L_ungespannt=elmatrix(:,5)'	                                                    ; % Vektor der ungespannten Elementlaengen
dof         =3*size(nkoord,1)                                                   ; % Bestimmung dof (3*max.KnotenNr.)
L_init      =vecnorm((nkoord(elmatrix(:,3),:)-nkoord(elmatrix(:,2),:))')        ; % Vektor der Elementlaengen in der Initialkonfiguration
u_vek       =nkoord-nkoord										                ; % Knotenverschiebungen sind Null
pos_vek     =nkoord								    		                    ; % Knotenpositionen entsprechend Initialkonfiguration 
fest0       =reshape(fest',[1,numel(fest)])                                     ; % Umformung Vektor der geometrischen Randbedingungen 
erg1=zeros(aufl2,numel(nkoord)+1)                                               ; % Initialisierung Ergebnisvektor
erg2=zeros(aufl2,3) ; erg3=zeros(2,size(elmatrix,1)+1)                          ; % Initialisierung Ergebnisvektoren

%%%%%%%%%% Inkremente - ANFANG %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for j=1:aufl2												
    L_vek=(L_ungespannt+(L_init-L_ungespannt)*(aufl2-j)/aufl2)                  ; % Vektor der fuer das Lastinkrement vorgegebenen Elementlaengen
    
	%%%%%%%%%% Iterationen - ANFANG %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:aufl
	
        %%%%%%%%% Systemgleichung generieren %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	    K=zeros(dof,dof) ; F_vek =zeros(dof,1) ;  du_vek=zeros(dof,1) ;	%Felem_vek=zeros(dof,1) ; F_vek=zeros(dof*3,1)	    ; % Initialisierung
        L0_vek   = vecnorm((pos_vek(elmatrix(:,3),:)-pos_vek(elmatrix(:,2),:))'); % Vektor der aktuellen Elementlaengen
	    wi       = (pos_vek(elmatrix(:,3),:)-pos_vek(elmatrix(:,2),:))./L0_vek' ; % Vektor der Elementwinkel
		Felem_vek= k_vek.*(L0_vek-L_vek)						                ; % Vektor der aktuellen Elementkraefte

        for ii=1:1:size(elmatrix,1)
            Ke=k_vek(ii)*wi(ii,:)'*wi(ii,:)+...
               Felem_vek(ii)/L0_vek(ii)*(eye(3)-wi(ii,:)'*wi(ii,:))             ; % elastische + geom. Elementsteifigkeitsuntermatrix des Elementes
            a=(elmatrix(ii,2)-1)*3+1;    b=(elmatrix(ii,3)-1)*3+1	            ; % KnotenNr. des Elementes in Variablen a und b speichern

		    K(a:a+2,a:a+2)=K(a:a+2,a:a+2)+Ke       					            ; % Aufsummieren der einzelnen erw. El.Steif.Matrizen
            K(a:a+2,b:b+2)=K(a:a+2,b:b+2)-Ke       					            ;
            K(b:b+2,a:a+2)=K(b:b+2,a:a+2)-Ke       					            ;
            K(b:b+2,b:b+2)=K(b:b+2,b:b+2)+Ke       					            ;

            F_vek(a:a+2)=F_vek(a:a+2)+(Felem_vek(ii)*wi(ii,:)') ; % Lastvektor - Vektor der inneren Elementkraefte generieren
            F_vek(b:b+2)=F_vek(b:b+2)-(Felem_vek(ii)*wi(ii,:)') ;
        end												         

   	    %%%%%%%%% reduzierte Systemgleichung aufloesen %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        du_vek(fest0~=0) = K(fest0~=0,fest0~=0) \ F_vek(fest0~=0)               ; % GLS loesen

	    %%%%%%%%% Verschiebungsergebnisse ordnen %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        du_vek =reshape(du_vek,[size(nkoord,2),size(nkoord,1)])'                ; % Umformung von du_vek
	    u_vek  =u_vek+du_vek										            ; % Verschiebungsvektor   aktualisieren
	    pos_vek=nkoord+u_vek									                ; % Knotenpositionsvektor aktualisieren
        dusum  =sum(vecnorm(du_vek'))                                           ; % Summe der Betraege der Knotenverschiebungsinkremente
     
	    %%%%%%%%% Kraefteungleichgewicht bestimmen %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Fu_vek =zeros(dof,1)                                                    ; % Initialisierung Kraefteungleichgewichtsvektor
        L0_vek=vecnorm((pos_vek(elmatrix(:,3),:)-pos_vek(elmatrix(:,2),:))')    ; % Vektor der aktuellen Elementlaengen
   	    Felem_vek=k_vek.*(L0_vek-L_vek)								            ; % aktuelle innere Elementkraft
        wi       = (pos_vek(elmatrix(:,3),:)-pos_vek(elmatrix(:,2),:))./L0_vek' ; % Vektor der Elementwinkel
        for ii=1:1:size(elmatrix,1)
            a=(elmatrix(ii,2)-1)*3+1;    b=(elmatrix(ii,3)-1)*3+1	            ; % KnotenNr. des Elementes in Variablen a und b speichern
            Fu_vek(a:a+2)=Fu_vek(a:a+2)+Felem_vek(ii)*wi(ii,:)'                 ; % Vektor der Knotenkräfte generieren
            Fu_vek(b:b+2)=Fu_vek(b:b+2)-Felem_vek(ii)*wi(ii,:)'                 ;
        end
        
        Fu_vek=reshape(Fu_vek,[size(nkoord,2),size(nkoord,1)])'                 ; % Umformung von Fu_vek

        %Fusum=sum(vecnorm(Fu_vek'))								            ; % Summe der Betraege der Kraefteungleichgewichte - aeussere Last und Lagerreaktionen im Ergebnis in Fusum sichtbar
        Fusum=norm(sum(Fu_vek))                                                 ; % Summe der Betraege der Kraefteungleichgewichte - aeussere Last und LR heben sich auf diese Weise auf

	    %%%%%%%%% Zwischenergebnisse speichern %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	    %erg ((j-1)*aufl+i,:)=[(j-1)*aufl+i j reshape(pos_vek,[1,numel(nkoord)])]; %
	    %ergu((j-1)*aufl+i,:)=[(j-1)*aufl+i j Fusum         dusum               ]; %

	    %%%%%%%%% vorzeitiges Abbruchkriterium %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	    if Fusum <=kraftkriterium && dusum <=verschiebungskriterium; break; end	  % Konvergenzkriterium (vorzeitige Konvergenz)

    end
	%%%%%%%%%% Iterationen - ENDE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 	ggl=0; if eigs(K(fest0~=0,fest0~=0),1,'smallestabs')>0; ggl=1; end				          % stat. Stabilitaet ueberpruefen
	konvkrit=0; if Fusum <= kraftkriterium && dusum <= verschiebungskriterium; konvkrit=1; end % Erfuellung der Konvergenzkriterien ueberpruefen

	erg1(j,:)=[j reshape(pos_vek,[1,numel(nkoord)])]                            ; % Ergebnisse Lastinkremente abspeichern
    erg2(j,:)=[j ggl konvkrit] ; erg3(j,:)=[j sign(Felem_vek)];

end													
%%%%%%%%%% Inkremente - ENDE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% zeit=toc;
% %%%%%%%%%% Beurteilung der Stabilitaet und Konvergenz %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if ggl==1 && konvkrit==1
%     disp(['Rechenzeit Formfindung: ' num2str(zeit) ' s, Formfindung: Stabile GGK, Konvergenz OK'])
% elseif ggl==0 && konvkrit==1
%     disp(['Rechenzeit Formfindung: ' num2str(zeit) ' s, Formfindung: INSTABILE GGK, Konvergenz OK'])
% elseif ggl==1 && konvkrit==0
%     disp(['Rechenzeit Formfindung: ' num2str(zeit) ' s, Formfindung: Stabile GGK, Konvergenz NICHT OK'])
% elseif ggl==0 && konvkrit==0
%     disp(['Rechenzeit Formfindung: ' num2str(zeit) ' s, Formfindung: INSTABILE GGK, Konvergenz NICHT OK'])
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Loading after Form finding / Kraftaufbringung nach der Formfindung
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic
%%%%%%%%%% Initialisierung %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k_vek       =elmatrix(:,4)'	                                                    ; % Vektor der Federsteifigkeiten
L_ungespannt=elmatrix(:,5)'	                                                    ; % Vektor der ungespannten Elementlaengen
dof         =3*size(nkoord,1)                                                   ; % Bestimmung dof (3*max.KnotenNr.)
L_init      =vecnorm((pos_vek(elmatrix(:,3),:)-pos_vek(elmatrix(:,2),:))')      ; % Vektor der Elementlaengen in der Initialkonfiguration
%u_vek       =nkoord-nkoord										                ; % Knotenverschiebungen sind Null
%pos_vek     =nkoord								    		                ; % Knotenpositionen entsprechend Initialkonfiguration 
fest0       =reshape(fest',[1,numel(fest)])                                     ; % Umformung Vektor der geometrischen Randbedingungen 
erg1=zeros(aufl2,numel(nkoord)+1)                                               ; % Initialisierung Ergebnisvektor
erg2=zeros(aufl2,3) ; erg3=zeros(2,size(elmatrix,1)+1)                          ; % Initialisierung Ergebnisvektoren

%%%%%%%%%% Inkremente - ANFANG %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for j=1:aufl2												
    L_vek=L_ungespannt                                                            ; % Vektor der fuer das Lastinkrement vorgegebenen Elementlaengen
    Fext_vek_inkr=Fext_vek*j/aufl2                                              ; % Vektor der fuer das Lastinkrement vorgegebenen Knotenkräfte

	%%%%%%%%%% Iterationen - ANFANG %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:aufl
	
        %%%%%%%%% Systemgleichung generieren %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	    K=zeros(dof,dof) ; F_vek =zeros(dof,1) ;  du_vek=zeros(dof,1) ;	%Felem_vek=zeros(dof,1) ; F_vek=zeros(dof*3,1)	    ; % Initialisierung
        L0_vek   = vecnorm((pos_vek(elmatrix(:,3),:)-pos_vek(elmatrix(:,2),:))'); % Vektor der aktuellen Elementlaengen
	    wi       = (pos_vek(elmatrix(:,3),:)-pos_vek(elmatrix(:,2),:))./L0_vek' ; % Vektor der Elementwinkel
		Felem_vek= k_vek.*(L0_vek-L_vek)						                ; % Vektor der aktuellen Elementkraefte

        for ii=1:1:size(elmatrix,1)
            Ke=k_vek(ii)*wi(ii,:)'*wi(ii,:)+...
               Felem_vek(ii)/L0_vek(ii)*(eye(3)-wi(ii,:)'*wi(ii,:))             ; % elastische + geom. Elementsteifigkeitsuntermatrix des Elementes
            a=(elmatrix(ii,2)-1)*3+1;    b=(elmatrix(ii,3)-1)*3+1	            ; % KnotenNr. des Elementes in Variablen a und b speichern

		    K(a:a+2,a:a+2)=K(a:a+2,a:a+2)+Ke       					            ; % Aufsummieren der einzelnen erw. El.Steif.Matrizen
            K(a:a+2,b:b+2)=K(a:a+2,b:b+2)-Ke       					            ;
            K(b:b+2,a:a+2)=K(b:b+2,a:a+2)-Ke       					            ;
            K(b:b+2,b:b+2)=K(b:b+2,b:b+2)+Ke       					            ;

            F_vek(a:a+2)=F_vek(a:a+2)+(Felem_vek(ii)*wi(ii,:)')                 ; % Lastvektor - Vektor der inneren Elementkraefte generieren
            F_vek(b:b+2)=F_vek(b:b+2)-(Felem_vek(ii)*wi(ii,:)')                 ;
        end												         
        F_vek=F_vek+Fext_vek_inkr                                               ; % Addition der inkrementellen Last              

   	    %%%%%%%%% reduzierte Systemgleichung aufloesen %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        du_vek(fest0~=0) = K(fest0~=0,fest0~=0) \ F_vek(fest0~=0)               ; % GLS loesen

	    %%%%%%%%% Verschiebungsergebnisse ordnen %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        du_vek =reshape(du_vek,[size(nkoord,2),size(nkoord,1)])'                ; % Umformung von du_vek
	    u_vek  =u_vek+du_vek										            ; % Verschiebungsvektor   aktualisieren
	    pos_vek=nkoord+u_vek									                ; % Knotenpositionsvektor aktualisieren
        dusum  =sum(vecnorm(du_vek'))                                           ; % Summe der Betraege der Knotenverschiebungsinkremente
     
	    %%%%%%%%% Kraefteungleichgewicht bestimmen %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Fu_vek =zeros(dof,1)                                                    ; % Initialisierung Kraefteungleichgewichtsvektor
        L0_vek=vecnorm((pos_vek(elmatrix(:,3),:)-pos_vek(elmatrix(:,2),:))')    ; % Vektor der aktuellen Elementlaengen
   	    Felem_vek=k_vek.*(L0_vek-L_vek)								            ; % aktuelle innere Elementkraft
        wi       = (pos_vek(elmatrix(:,3),:)-pos_vek(elmatrix(:,2),:))./L0_vek' ; % Vektor der Elementwinkel
        for ii=1:1:size(elmatrix,1)
            a=(elmatrix(ii,2)-1)*3+1;    b=(elmatrix(ii,3)-1)*3+1	            ; % KnotenNr. des Elementes in Variablen a und b speichern
            Fu_vek(a:a+2)=Fu_vek(a:a+2)+Felem_vek(ii)*wi(ii,:)'                 ; % Vektor der Knotenkräfte generieren
            Fu_vek(b:b+2)=Fu_vek(b:b+2)-Felem_vek(ii)*wi(ii,:)'                 ;
        end
        
        Fu_vek=reshape(Fu_vek,[size(nkoord,2),size(nkoord,1)])'                 ; % Umformung von Fu_vek

        %Fusum=sum(vecnorm(Fu_vek'))								            ; % Summe der Betraege der Kraefteungleichgewichte - aeussere Last und Lagerreaktionen im Ergebnis in Fusum sichtbar
        Fusum=norm(sum(Fu_vek))                                                 ; % Summe der Betraege der Kraefteungleichgewichte - aeussere Last und LR heben sich auf diese Weise auf

	    %%%%%%%%% Zwischenergebnisse speichern %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	    %erg ((j-1)*aufl+i,:)=[(j-1)*aufl+i j reshape(pos_vek,[1,numel(nkoord)])]; %
	    %ergu((j-1)*aufl+i,:)=[(j-1)*aufl+i j Fusum         dusum               ]; %

	    %%%%%%%%% vorzeitiges Abbruchkriterium %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	    if Fusum <=kraftkriterium && dusum <=verschiebungskriterium; break; end	  % Konvergenzkriterium (vorzeitige Konvergenz)

    end
	%%%%%%%%%% Iterationen - ENDE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 	ggl=0; if eigs(K(fest0~=0,fest0~=0),1,'smallestabs')>0; ggl=1; end				          % stat. Stabilitaet ueberpruefen
	konvkrit=0; if Fusum <= kraftkriterium && dusum <= verschiebungskriterium; konvkrit=1; end % Erfuellung der Konvergenzkriterien ueberpruefen

	erg1(j,:)=[j reshape(pos_vek,[1,numel(nkoord)])]                            ; % Ergebnisse Lastinkremente abspeichern
    erg2(j,:)=[j ggl konvkrit] ; erg3(j,:)=[j sign(Felem_vek)];

end													
%%%%%%%%%% Inkremente - ENDE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% zeit=toc;
% %%%%%%%%%% Beurteilung der Stabilitaet und Konvergenz %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if ggl==1 && konvkrit==1
%     disp(['Rechenzeit Lastaufbringung: ' num2str(zeit) ' s, Formfindung: Stabile GGK, Konvergenz OK'])
% elseif ggl==0 && konvkrit==1
%     disp(['Rechenzeit Lastaufbringung: ' num2str(zeit) ' s, Formfindung: INSTABILE GGK, Konvergenz OK'])
% elseif ggl==1 && konvkrit==0
%     disp(['Rechenzeit Lastaufbringung: ' num2str(zeit) ' s, Formfindung: Stabile GGK, Konvergenz NICHT OK'])
% elseif ggl==0 && konvkrit==0
%     disp(['Rechenzeit Lastaufbringung: ' num2str(zeit) ' s, Formfindung: INSTABILE GGK, Konvergenz NICHT OK'])
% end













%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Ausgabe Ergebnisse Konvergenz / Knotenpositionen / Stabilitaet der GGL / Federkraefte %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%pos_vek			; % Vektor der Knotenpositionen in der Gleichgewichtskonfiguration
%ggl				; % statische Stabilitaet der Gleichgewichtskonfiguration (1: stabil, 0: instabil)
%elementkraft_vek=Felem_vek	; % Vektor der inneren Elementkraefte in der Gleichgewichtskonfiguration
%konvkrit			; % Konvergenz am Ende der Berechnung (1: konvergiert, 0: nicht konvergiert)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Ergebnisgroessen %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pos_vek           % Matrix der Knotenpositionen in der Gleichgewichtskonfiguration, 1.Zeile alle x-Koordinaten; 2. Zeile alle y-Koordinaten; 3. Zeile alle z-Koordinaten
% erg               % Matrix der Knotenpositionen in der Gleichgewichtskonfiguration je Iteration; Zeilennummer = Iterationsnummer; Zeile Knotennummern [Iteration Inkrement x_1 y_1 z_1 x_2 ....]
% ergu              % Matrix der Konvergenzkriterien je Iteration
% erg1              % Matrix der Knotenpositionen in der Gleichgewichtskonfiguration nach jedem Inkrement; Zeilennummer = Inkrementnummer; Zeile Knotennummern [x_1 y_1 z_1 x_2 ....]
% erg2              % Matrix der statischen Stabilitaet und der Konvegenz je Inkrement; [ggl, konvkrit]
% erg3              % Matrix der Elementkraeftevorzeichen je Inkrement
% ggl			    % statische Stabilitaet der Gleichgewichtskonfiguration (1: stabil, 0: instabil)
% konvkrit		    % Konvergenz am Ende der Berechnung (1: konvergiert, 0: nicht konvergiert)

% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%% Ergebnisdarstellung %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% lws=3; lws2=3; lwf=1; lwf2=1; f2=[0 0 0];  f1=[0.7 0.7 0.7]						; % Darstellungsparameter
% 
% %%%%%%%%%%%%%%%%% BILD 1 PLOTTEN - Initialkonfiguration, Gleichgewichtskonfiguration %%%%%%%%%%%%%%%%%%%%
% 
% xlage=14; ylage=2                                     							; % Position auf Bildschirm
% ho=10; ve=10; xan=0.125; yan=0.125; xen=0.025; yen=0.025  						; % Bildgroesse, Diagrammgroesse innerhalb des Bildes
%    fh=figure(UN,CM,POS,[xlage,ylage,ho,ve],C,'w')                                               	; % Bildlage
%    set(gca,UN,CM,POS,[xan*ho yan*ve (1-xen-xan)*ho (1-yen-yan)*ve],FN,tim,FS,fsiz,C,'w')     		; % Diagrammposition, Schrift
%    set(gca,'XScale','linear','XDir','normal'); set(gca,XACH,'bottom',YACH,'left')             		; % Achsenskalierung, Achsenlage
%    set(gcf,PUN,CM,PS,[ho ve],PPOS,[0 0 ho ve],C,'w'); set(gca,LW,lwgca,GLS,gls)              		; % Papiergroesse, Rahmenfarbe, Gridlinestyle
% 
%    xan=1; yan=1; xen=0.25; yen=0.25									;
%    subplot(POS,[xan/ho yan/ve (ho-xan-xen)/ho (ve-yen-yan)/ve],FN,tim,FS,fsiz,C,'w'); hold on		;
%    set(gca,FN,tim,FS,fsiz,C,'w'); 
% 
% video_test=VideoWriter('animation_test.avi');  %%% Videoerstellung
% video_test.FrameRate=15;                            %%%
% video_test.Quality=90;                              %%%
% open(video_test);                                   %%%
% pause(0.5)
% for iii=1:aufl2;
% clf('reset'); set(gcf,C,'w'); 
% pos_vek2=erg1(iii,2:19); pos_vek=reshape(pos_vek2,[6,3]); 
% 
% 
%         for i=1:size(elmatrix,1); hold on
% 	    if k_vek(i)<1000; lw=1; else; lw=2; end							
%  	    plot3([nkoord(elmatrix(i,3),1) nkoord(elmatrix(i,2),1)],...				      % plot Initialkonfiguration
% 	          [nkoord(elmatrix(i,3),2) nkoord(elmatrix(i,2),2)],...				    
% 	          [nkoord(elmatrix(i,3),3) nkoord(elmatrix(i,2),3)],'k-',LW,lw,C,f1)
%         end
%         for i=1:size(elmatrix,1); hold on
% 	    if k_vek(i)<1000; lw=1; else; lw=2; end							
%               plot3([pos_vek(elmatrix(i,3),1) pos_vek(elmatrix(i,2),1)],...				  % plot Gleichgewichtskonfiguration
% 	          [pos_vek(elmatrix(i,3),2) pos_vek(elmatrix(i,2),2)],...				
% 	          [pos_vek(elmatrix(i,3),3) pos_vek(elmatrix(i,2),3)],'k-',LW,lw,C,f2)	
%         end
% 
% 	box 'on'; axis equal; xlim([-0.1,1.1]); ylim([-0.6,0.6]); set(gca,FN,tim,FS,fsiz,C,'w')
% 	view(0,90);	
% 
% text(0.5,0.5,num2str(iii),FN,tim,FS,fsiz)
% frame=getframe(gcf);            %%%
% writeVideo(video_test,frame);   %%% Inhalt des Bildes in Videodatei mit dazunehmen
% %pause(0.05)
% 
% dhilf=pos_vek-nkoord;
% d1=dhilf(3,1); d2=dhilf(5,2);
% dwerte(iii,:)=[Kraftvonaussen(1,1)*iii/aufl2 d1 d2];
% end;
% 
% close(video_test);              %%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% final results for nodal positions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ggl==1 && konvkrit==1
    output = 1; 
nodal_positions=pos_vek;   % matrix nodal positions exists, if equilibrium is stable and convergence criteria are fulfilled
else output =0;
end